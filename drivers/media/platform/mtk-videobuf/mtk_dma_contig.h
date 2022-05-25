/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Rick Chang <rick.chang@mediatek.com>
 */

#ifndef _MEDIA_MTK_DMA_CONTIG_H
#define _MEDIA_MTK_DMA_CONTIG_H

#include <media/videobuf2-v4l2.h>
#include <linux/dma-mapping.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-memops.h>

struct vb2_dc_buf {
	struct device			*dev;
	void				*vaddr;
	unsigned long			size;
	void				*cookie;
	dma_addr_t			dma_addr;
	unsigned long			attrs;
	enum dma_data_direction		dma_dir;
	struct sg_table			*dma_sgt;
	struct frame_vector		*vec;

	/* MMAP related */
	struct vb2_vmarea_handler	handler;
	atomic_t			refcount;
	struct sg_table			*sgt_base;

	/* DMABUF related */
	struct dma_buf_attachment	*db_attach;
};

static inline dma_addr_t
mtk_dma_contig_plane_dma_addr(struct vb2_buffer *vb, unsigned int plane_no)
{
	dma_addr_t *addr = vb2_plane_cookie(vb, plane_no);

	return *addr;
}

int mtk_dma_contig_set_max_seg_size(struct device *dev, unsigned int size);
void mtk_dma_contig_clear_max_seg_size(struct device *dev);
void mtk_dma_contig_set_secure_mode(struct device *dev, int secure_mode);

extern const struct vb2_mem_ops mtk_dma_contig_memops;

#endif
