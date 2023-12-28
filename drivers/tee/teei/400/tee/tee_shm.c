// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015-2016, Linaro Limited
 * Copyright (c) 2015-2019, MICROTRUST Incorporated
 *
 *
 */
#include <linux/version.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/fdtable.h>
#include <linux/idr.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>
#include <tee_drv.h>
#include <imsg_log.h>
#include "tee_private.h"

/* extra references appended to shm object for registered shared memory */
struct tee_shm_dmabuf_ref {
	struct tee_shm shm;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
};

static void tee_shm_release(struct tee_shm *shm)
{
	struct tee_device *teedev = shm->teedev;

	mutex_lock(&teedev->mutex);

	if (shm->id >= 0)
		idr_remove(&teedev->idr, shm->id);

	if (shm->ctx)
		list_del(&shm->link);

	mutex_unlock(&teedev->mutex);

	if (shm->flags & TEE_SHM_EXT_DMA_BUF) {
		struct tee_shm_dmabuf_ref *ref;

		ref = container_of(shm, struct tee_shm_dmabuf_ref, shm);
		dma_buf_unmap_attachment(ref->attach, ref->sgt,
					 DMA_BIDIRECTIONAL);
		dma_buf_detach(shm->dmabuf, ref->attach);
		dma_buf_put(ref->dmabuf);
	} else {
		struct tee_shm_pool_mgr *poolm;

		if ((shm->flags & TEE_SHM_DMA_BUF) || (shm->flags & TEE_SHM_DMA_KERN_BUF))
			poolm = &teedev->pool->dma_buf_mgr;
		else
			poolm = &teedev->pool->private_mgr;

		poolm->ops->free(poolm, shm);
	}

	kfree(shm);
	isee_device_put(teedev);
}

static struct sg_table *tee_shm_op_map_dma_buf(struct dma_buf_attachment
			*attach, enum dma_data_direction dir)
{
	return NULL;
}

static void tee_shm_op_unmap_dma_buf(struct dma_buf_attachment *attach,
				     struct sg_table *table,
				     enum dma_data_direction dir)
{
}

static void tee_shm_op_release(struct dma_buf *dmabuf)
{
	struct tee_shm *shm = dmabuf->priv;

	tee_shm_release(shm);
}

static int tee_shm_op_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct tee_shm *shm = dmabuf->priv;
	size_t size = vma->vm_end - vma->vm_start;

	if (size > shm->size) {
		IMSG_INFO("Invalid memory mapping size.\n");
		return -EINVAL;
	}

	return remap_pfn_range(vma, vma->vm_start, shm->paddr >> PAGE_SHIFT,
			       size, vma->vm_page_prot);
}

static struct dma_buf_ops tee_shm_dma_buf_ops = {
	.map_dma_buf = tee_shm_op_map_dma_buf,
	.unmap_dma_buf = tee_shm_op_unmap_dma_buf,
	.release = tee_shm_op_release,
	.mmap = tee_shm_op_mmap,
};

struct tee_shm *isee_shm_kalloc(struct tee_context *ctx, size_t size, u32 flags)
{
	struct tee_device *teedev = ctx->teedev;
	struct tee_shm_pool_mgr *poolm = NULL;
	struct tee_shm *shm;
	void *ret;
	int rc;

	if (!(flags & TEE_SHM_MAPPED)) {
		IMSG_ERROR(
			"only mapped allocations supported\n");
		return ERR_PTR(-EINVAL);
	}

	if ((flags & ~(TEE_SHM_MAPPED | TEE_SHM_DMA_KERN_BUF))) {
		IMSG_ERROR("invalid shm flags 0x%x", flags);
		return ERR_PTR(-EINVAL);
	}

	if (!isee_device_get(teedev))
		return ERR_PTR(-EINVAL);

	if (!teedev->pool) {
		/* teedev has been detached from driver */
		ret = ERR_PTR(-EINVAL);
		goto err_dev_put;
	}

	shm = kzalloc(sizeof(*shm), GFP_KERNEL);
	if (!shm) {
		ret = ERR_PTR(-ENOMEM);
		goto err_dev_put;
	}

	shm->flags = flags;
	shm->teedev = teedev;
	shm->ctx = ctx;
	if (flags & TEE_SHM_DMA_KERN_BUF)
		poolm = &teedev->pool->dma_buf_mgr;
	else
		poolm = &teedev->pool->private_mgr;

	rc = poolm->ops->alloc(poolm, shm, size);
	if (rc) {
		ret = ERR_PTR(rc);
		goto err_kfree;
	}

	mutex_lock(&teedev->mutex);
	shm->id = idr_alloc(&teedev->idr, shm, 1, 0, GFP_KERNEL);
	mutex_unlock(&teedev->mutex);
	if (shm->id < 0) {
		ret = ERR_PTR(shm->id);
		goto err_pool_free;
	}

	mutex_lock(&teedev->mutex);
	list_add_tail(&shm->link, &ctx->list_shm);
	mutex_unlock(&teedev->mutex);

	return shm;

err_pool_free:
	poolm->ops->free(poolm, shm);
err_kfree:
	kfree(shm);
err_dev_put:
	isee_device_put(teedev);
	return ret;
}
EXPORT_SYMBOL_GPL(isee_shm_kalloc);

void isee_shm_kfree(struct tee_shm *shm)
{
	tee_shm_release(shm);
}
EXPORT_SYMBOL_GPL(isee_shm_kfree);



/**
 * isee_shm_alloc() - Allocate shared memory
 * @ctx:	Context that allocates the shared memory
 * @size:	Requested size of shared memory
 * @flags:	Flags setting properties for the requested shared memory.
 *
 * Memory allocated as global shared memory is automatically freed when the
 * TEE file pointer is closed. The @flags field uses the bits defined by
 * TEE_SHM_* in <linux/tee_drv.h>. TEE_SHM_MAPPED must currently always be
 * set. If TEE_SHM_DMA_BUF global shared memory will be allocated and
 * associated with a dma-buf handle, else driver private memory.
 */
struct tee_shm *isee_shm_alloc_noid(struct tee_context *ctx, size_t size, u32 flags)
{
	struct tee_device *teedev = ctx->teedev;
	struct tee_shm_pool_mgr *poolm = NULL;
	struct tee_shm *shm;
	void *ret;
	int rc;

	if (!(flags & TEE_SHM_MAPPED)) {
		IMSG_ERROR(
			"only mapped allocations supported\n");
		return ERR_PTR(-EINVAL);
	}

	if ((flags & ~(TEE_SHM_MAPPED | TEE_SHM_DMA_BUF))) {
		IMSG_ERROR("invalid shm flags 0x%x", flags);
		return ERR_PTR(-EINVAL);
	}

	if (!isee_device_get(teedev))
		return ERR_PTR(-EINVAL);

	if (!teedev->pool) {
		/* teedev has been detached from driver */
		ret = ERR_PTR(-EINVAL);
		goto err_dev_put;
	}

	shm = kzalloc(sizeof(*shm), GFP_KERNEL);
	if (!shm) {
		ret = ERR_PTR(-ENOMEM);
		goto err_dev_put;
	}

	shm->flags = flags;
	shm->teedev = teedev;
	shm->ctx = ctx;
	if (flags & TEE_SHM_DMA_BUF)
		poolm = &teedev->pool->dma_buf_mgr;
	else
		poolm = &teedev->pool->private_mgr;

	rc = poolm->ops->alloc(poolm, shm, size);
	if (rc) {
		ret = ERR_PTR(rc);
		goto err_kfree;
	}

	shm->id = -1;

	if (flags & TEE_SHM_DMA_BUF) {
#if KERNEL_VERSION(4, 4, 1) <= LINUX_VERSION_CODE
		DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

		exp_info.ops = &tee_shm_dma_buf_ops;
		exp_info.size = shm->size;
		exp_info.flags = O_RDWR;
		exp_info.priv = shm;

		shm->dmabuf = dma_buf_export(&exp_info);
#else
		shm->dmabuf = dma_buf_export(shm, &tee_shm_dma_buf_ops,
						shm->size, O_RDWR, NULL);
#endif
		if (IS_ERR(shm->dmabuf)) {
			ret = ERR_CAST(shm->dmabuf);
			goto err_rem;
		}
	}
	mutex_lock(&teedev->mutex);
	list_add_tail(&shm->link, &ctx->list_shm);
	mutex_unlock(&teedev->mutex);

	return shm;
err_rem:
	poolm->ops->free(poolm, shm);
err_kfree:
	kfree(shm);
err_dev_put:
	isee_device_put(teedev);
	return ret;
}
EXPORT_SYMBOL_GPL(isee_shm_alloc_noid);


struct tee_shm *isee_shm_alloc(struct tee_context *ctx, size_t size, u32 flags)
{
	struct tee_device *teedev = ctx->teedev;
	struct tee_shm *shm = NULL;
	void *ret = NULL;

	shm = isee_shm_alloc_noid(ctx, size, flags);
	if (IS_ERR(shm)) {
		IMSG_ERROR("Failed to alloc shm %lld\n", PTR_ERR(shm));
		return shm;
	}

	mutex_lock(&teedev->mutex);
	shm->id = idr_alloc(&teedev->idr, shm, 1, 0, GFP_KERNEL);
	mutex_unlock(&teedev->mutex);

	if (shm->id < 0) {
		IMSG_ERROR("Failed to alloc shm_id %d\n", shm->id);
		ret = ERR_PTR(shm->id);
		goto err_shm_free;
	}

	return shm;

err_shm_free:
	tee_shm_release(shm);

	return ret;
}
EXPORT_SYMBOL_GPL(isee_shm_alloc);

/**
 * isee_shm_get_fd() - Increase reference count and return file descriptor
 * @shm:	Shared memory handle
 * @returns user space file descriptor to shared memory
 */
int isee_shm_get_fd(struct tee_shm *shm)
{
	int fd;

	if (!(shm->flags & TEE_SHM_DMA_BUF))
		return -EINVAL;

	fd = dma_buf_fd(shm->dmabuf, O_CLOEXEC);
	if (fd >= 0)
		get_dma_buf(shm->dmabuf);
	return fd;
}

/**
 * isee_shm_free() - Free shared memory
 * @shm:	Handle to shared memory to free
 */
void isee_shm_free(struct tee_shm *shm)
{
	/*
	 * dma_buf_put() decreases the dmabuf reference counter and will
	 * call tee_shm_release() when the last reference is gone.
	 *
	 * In the case of driver private memory we call tee_shm_release
	 * directly instead as it doesn't have a reference counter.
	 */
	if (shm->flags & TEE_SHM_DMA_BUF)
		dma_buf_put(shm->dmabuf);
	else
		tee_shm_release(shm);
}
EXPORT_SYMBOL_GPL(isee_shm_free);

/**
 * isee_shm_va2pa() - Get physical address of a virtual address
 * @shm:	Shared memory handle
 * @va:		Virtual address to tranlsate
 * @pa:		Returned physical address
 * @returns 0 on success and < 0 on failure
 */
int isee_shm_va2pa(struct tee_shm *shm, void *va, phys_addr_t *pa)
{
	if (!(shm->flags & TEE_SHM_MAPPED))
		return -EINVAL;
	/* Check that we're in the range of the shm */
	if ((char *)va < (char *)shm->kaddr)
		return -EINVAL;
	if ((char *)va >= ((char *)shm->kaddr + shm->size))
		return -EINVAL;

	return isee_shm_get_pa(
			shm, (unsigned long)va - (unsigned long)shm->kaddr, pa);
}
EXPORT_SYMBOL_GPL(isee_shm_va2pa);

/**
 * isee_shm_pa2va() - Get virtual address of a physical address
 * @shm:	Shared memory handle
 * @pa:		Physical address to tranlsate
 * @va:		Returned virtual address
 * @returns 0 on success and < 0 on failure
 */
int isee_shm_pa2va(struct tee_shm *shm, phys_addr_t pa, void **va)
{
	if (!(shm->flags & TEE_SHM_MAPPED))
		return -EINVAL;
	/* Check that we're in the range of the shm */
	if (pa < shm->paddr)
		return -EINVAL;
	if (pa >= (shm->paddr + shm->size))
		return -EINVAL;

	if (va) {
		void *v = isee_shm_get_va(shm, pa - shm->paddr);

		if (IS_ERR(v))
			return PTR_ERR(v);
		*va = v;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(isee_shm_pa2va);

/**
 * isee_shm_get_va() - Get virtual address of a shared memory plus an offset
 * @shm:	Shared memory handle
 * @offs:	Offset from start of this shared memory
 * @returns virtual address of the shared memory + offs if offs is within
 *	the bounds of this shared memory, else an ERR_PTR
 */
void *isee_shm_get_va(struct tee_shm *shm, size_t offs)
{
	if (!(shm->flags & TEE_SHM_MAPPED))
		return ERR_PTR(-EINVAL);
	if (offs >= shm->size)
		return ERR_PTR(-EINVAL);
	return (char *)shm->kaddr + offs;
}
EXPORT_SYMBOL_GPL(isee_shm_get_va);

/**
 * isee_shm_get_pa() - Get physical address of a shared memory plus an offset
 * @shm:	Shared memory handle
 * @offs:	Offset from start of this shared memory
 * @pa:		Physical address to return
 * @returns 0 if offs is within the bounds of this shared memory, else an
 *	error code.
 */
int isee_shm_get_pa(struct tee_shm *shm, size_t offs, phys_addr_t *pa)
{
	if (offs >= shm->size)
		return -EINVAL;
	if (pa)
		*pa = shm->paddr + offs;
	return 0;
}
EXPORT_SYMBOL_GPL(isee_shm_get_pa);

/**
 * isee_shm_get_from_id() - Find shared memory object and increase reference
 * count
 * @ctx:	Context owning the shared memory
 * @id:		Id of shared memory object
 * @returns a pointer to 'struct tee_shm' on success or an ERR_PTR on failure
 */
struct tee_shm *isee_shm_get_from_id(struct tee_context *ctx, int id)
{
	struct tee_device *teedev;
	struct tee_shm *shm;

	if (!ctx)
		return ERR_PTR(-EINVAL);

	teedev = ctx->teedev;
	mutex_lock(&teedev->mutex);
	shm = idr_find(&teedev->idr, id);
	if (!shm || shm->ctx != ctx)
		shm = ERR_PTR(-EINVAL);
	else if (shm->flags & TEE_SHM_DMA_BUF)
		get_dma_buf(shm->dmabuf);
	mutex_unlock(&teedev->mutex);
	return shm;
}
EXPORT_SYMBOL_GPL(isee_shm_get_from_id);

/**
 * isee_shm_get_id() - Get id of a shared memory object
 * @shm:	Shared memory handle
 * @returns id
 */
int isee_shm_get_id(struct tee_shm *shm)
{
	return shm->id;
}
EXPORT_SYMBOL_GPL(isee_shm_get_id);

/**
 * isee_shm_put() - Decrease reference count on a shared memory handle
 * @shm:	Shared memory handle
 */
void isee_shm_put(struct tee_shm *shm)
{
	if (shm->flags & TEE_SHM_DMA_BUF)
		dma_buf_put(shm->dmabuf);
}
EXPORT_SYMBOL_GPL(isee_shm_put);
