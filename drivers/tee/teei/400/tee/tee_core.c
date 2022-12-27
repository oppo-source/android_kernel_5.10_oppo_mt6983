// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015-2016, Linaro Limited
 * Copyright (c) 2015-2019, MICROTRUST Incorporated
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <tee_client_api.h>
#include <tee_drv.h>
#include <asm/cacheflush.h>
#include <imsg_log.h>
#include "tee_private.h"
#include "capi_proxy.h"
#include "teei_id.h"

#define TEE_NUM_DEVICES	32
#define MAX_NUM_PARAMS 4

#define TEE_IOCTL_PARAM_SIZE(x) (sizeof(struct tee_param) * (x))

/*
 * Unprivileged devices in the lower half range and privileged devices in
 * the upper half range.
 */
static DECLARE_BITMAP(dev_mask, TEE_NUM_DEVICES);
static DEFINE_SPINLOCK(driver_lock);

static struct class *tee_class;
static dev_t tee_devt;

/* helper to convert user pointers passed inside __aligned_u64 fields */
#ifndef u64_to_user_ptr
static void __user *u64_to_user_ptr(__u64 val)
{
	return (void __user *) (unsigned long) val;
}
#endif

static struct tee_context *teedev_open(struct tee_device *teedev)
{
	int rc;
	struct tee_context *ctx;

	if (!isee_device_get(teedev))
		return ERR_PTR(-EINVAL);

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		rc = -ENOMEM;
		goto err;
	}

	ctx->teedev = teedev;
	INIT_LIST_HEAD(&ctx->list_shm);
	rc = teedev->desc->ops->open(ctx);
	if (rc)
		goto err;

	return ctx;
err:
	kfree(ctx);
	isee_device_put(teedev);
	return ERR_PTR(rc);

}

static void teedev_close_context(struct tee_context *ctx)
{
	struct tee_shm *shm;

	mutex_lock(&ctx->mutex);

	ctx->teedev->desc->ops->release(ctx);
	mutex_lock(&ctx->teedev->mutex);
	list_for_each_entry(shm, &ctx->list_shm, link)
		shm->ctx = NULL;
	mutex_unlock(&ctx->teedev->mutex);
	isee_device_put(ctx->teedev);

	mutex_unlock(&ctx->mutex);

	kfree(ctx);
}

int tee_k_open(struct file *filp)
{
	struct tee_context *ctx;
	struct tee_device *teedev = NULL;

	teedev = isee_get_teedev();
	if (teedev == NULL) {
		IMSG_ERROR("Failed to get the teedev!\n");
		return -EINVAL;
	}

	ctx = teedev_open(teedev);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	mutex_init(&ctx->mutex);

	filp->private_data = ctx;
	return 0;
}

static int tee_open(struct inode *inode, struct file *filp)
{
	struct tee_context *ctx;

	ctx = teedev_open(container_of(inode->i_cdev, struct tee_device, cdev));
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	mutex_init(&ctx->mutex);

	filp->private_data = ctx;
	return 0;
}

int tee_k_release(struct file *filp)
{
	teedev_close_context(filp->private_data);
	return 0;
}

static int tee_release(struct inode *inode, struct file *filp)
{
	teedev_close_context(filp->private_data);
	return 0;
}

static int tee_ioctl_version(struct tee_context *ctx,
			     struct tee_ioctl_version_data __user *uvers)
{
	struct tee_ioctl_version_data vers;

	ctx->teedev->desc->ops->get_version(ctx->teedev, &vers);

	if (ctx->teedev->desc->flags & TEE_DESC_PRIVILEGED)
		vers.gen_caps |= TEE_GEN_CAP_PRIVILEGED;

	if (copy_to_user(uvers, &vers, sizeof(vers)))
		return -EFAULT;

	return 0;
}

#if IS_ENABLED(CONFIG_MICROTRUST_TEST_DRIVERS)

static inline void flush_shm_dcache(void *start, size_t len)
{
#ifdef CONFIG_ARM64
		__flush_dcache_area(start, len);
#else
		__cpuc_flush_dcache_area(start, len);
#endif
}

static inline void invalid_shm_dcache(void *start, size_t len)
{
	__Invalidate_Dcache_By_Area((unsigned long)start,
				(unsigned long)(start + len));
}

static int tee_ioctl_shm_kern_op(struct tee_context *ctx,
			       struct tee_ioctl_shm_kern_op_arg __user *udata)
{
	long ret = 0;

	struct tee_ioctl_shm_kern_op_arg data;
	struct tee_shm *shm;

	if (copy_from_user(&data, udata, sizeof(data)))
		return -EFAULT;


	shm = isee_shm_get_from_id(ctx, data.id);
	if (IS_ERR(shm))
		return PTR_ERR(shm);

	switch (data.opcode) {
	case TEE_IOCTL_SHM_KERN_OP_GET_PA:
		data.paddr = shm->paddr;
		break;
	case TEE_IOCTL_SHM_KERN_OP_FLUSH_CACHE:
		flush_shm_dcache(shm->kaddr, shm->size);
		break;
	case TEE_IOCTL_SHM_KERN_OP_INVALID_CACHE:
		invalid_shm_dcache(shm->kaddr, shm->size);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	if (copy_to_user(udata, &data, sizeof(data)))
		ret = -EFAULT;

	/*
	 * When user space closes the file descriptor the shared memory
	 * should be freed or if isee_shm_get_fd() failed then it will
	 * be freed immediately.
	 */
out:
	isee_shm_put(shm);

	return ret;
}

#endif

static int params_from_user(struct tee_context *ctx, struct tee_param *params,
			    size_t num_params,
			    struct tee_ioctl_param __user *uparams)
{
	size_t n;

	for (n = 0; n < num_params; n++) {
		struct tee_shm *shm;
		struct tee_ioctl_param ip;

		if (copy_from_user(&ip, uparams + n, sizeof(ip)))
			return -EFAULT;

		/* All unused attribute bits has to be zero */
		if (ip.attr & ~TEE_IOCTL_PARAM_ATTR_MASK)
			return -EINVAL;

		params[n].attr = ip.attr;
		switch (ip.attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) {
		case TEE_IOCTL_PARAM_ATTR_TYPE_NONE:
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT:
			break;
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT:
			params[n].u.value.a = ip.a;
			params[n].u.value.b = ip.b;
			params[n].u.value.c = ip.c;
			break;
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT:
			/*
			 * If we fail to get a pointer to a shared memory
			 * object (and increase the ref count) from an
			 * identifier we return an error. All pointers that
			 * has been added in params have an increased ref
			 * count. It's the callers responibility to do
			 * isee_shm_put() on all resolved pointers.
			 */
			shm = isee_shm_get_from_id(ctx, (int)(ip.c));
			if (IS_ERR(shm))
				return (int)(PTR_ERR(shm));

			if ((ip.a >= shm->size) || (ip.b > shm->size)
					|| ((ip.a + ip.b) > shm->size)) {
				IMSG_ERROR("Inval param in %s\n", __func__);
				return -EINVAL;
			}

			params[n].u.memref.shm_offs = ip.a;
			params[n].u.memref.size = ip.b;
			params[n].u.memref.shm = shm;
			break;
		default:
			/* Unknown attribute */
			return -EINVAL;
		}
	}
	return 0;
}

static int params_to_user(struct tee_ioctl_param __user *uparams,
			  size_t num_params, struct tee_param *params)
{
	size_t n;

	for (n = 0; n < num_params; n++) {
		struct tee_ioctl_param __user *up = uparams + n;
		struct tee_param *p = params + n;

		switch (p->attr) {
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT:
			if (put_user(p->u.value.a, &up->a) ||
			    put_user(p->u.value.b, &up->b) ||
			    put_user(p->u.value.c, &up->c))
				return -EFAULT;
			break;
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT:
			if (put_user((u64)p->u.memref.size, &up->b))
				return -EFAULT;
		default:
			break;
		}
	}
	return 0;
}

static int tee_ioctl_open_session(struct tee_context *ctx,
				  struct tee_ioctl_buf_data __user *ubuf)
{
	int rc;
	size_t n;
	struct tee_ioctl_buf_data buf;
	struct tee_ioctl_open_session_arg __user *uarg;
	struct tee_ioctl_open_session_arg arg;
	struct tee_ioctl_param __user *uparams = NULL;
	struct tee_param *params = NULL;
	bool have_session = false;

	if (!ctx->teedev->desc->ops->open_session)
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, sizeof(buf)))
		return -EFAULT;

	if (buf.buf_len > TEE_MAX_ARG_SIZE ||
	    buf.buf_len < sizeof(struct tee_ioctl_open_session_arg))
		return -EINVAL;

	uarg = u64_to_user_ptr(buf.buf_ptr);
	if (copy_from_user(&arg, uarg, sizeof(arg)))
		return -EFAULT;

	if (arg.num_params > MAX_NUM_PARAMS)
		return -EINVAL;

	if (sizeof(arg) + TEE_IOCTL_PARAM_SIZE(arg.num_params) != buf.buf_len)
		return -EINVAL;

	if (arg.num_params) {
		params = kcalloc(arg.num_params, sizeof(struct tee_param),
				 GFP_KERNEL);
		if (!params)
			return -ENOMEM;
		uparams = uarg->params;
		rc = params_from_user(ctx, params, (size_t)arg.num_params,
				 uparams);
		if (rc)
			goto out;
	}

	rc = ctx->teedev->desc->ops->open_session(ctx, &arg, params);
	if (rc)
		goto out;
	have_session = true;

	if (put_user(arg.session, &uarg->session) ||
	    put_user(arg.ret, &uarg->ret) ||
	    put_user(arg.ret_origin, &uarg->ret_origin)) {
		rc = -EFAULT;
		goto out;
	}
	rc = params_to_user(uparams, (size_t)arg.num_params, params);
out:
	/*
	 * If we've succeeded to open the session but failed to communicate
	 * it back to user space, close the session again to avoid leakage.
	 */
	if (rc && have_session && ctx->teedev->desc->ops->close_session)
		ctx->teedev->desc->ops->close_session(ctx, arg.session);

	if (params) {
		/* Decrease ref count for all valid shared memory pointers */
		for (n = 0; n < arg.num_params; n++)
			if (tee_param_is_memref(params + n) &&
			    params[n].u.memref.shm)
				isee_shm_put(params[n].u.memref.shm);
		kfree(params);
	}

	return rc;
}

static int tee_ioctl_invoke(struct tee_context *ctx,
			    struct tee_ioctl_buf_data __user *ubuf)
{
	int rc;
	size_t n;
	struct tee_ioctl_buf_data buf;
	struct tee_ioctl_invoke_arg __user *uarg;
	struct tee_ioctl_invoke_arg arg;
	struct tee_ioctl_param __user *uparams = NULL;
	struct tee_param *params = NULL;

	if (!ctx->teedev->desc->ops->invoke_func)
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, sizeof(buf)))
		return -EFAULT;

	if (buf.buf_len > TEE_MAX_ARG_SIZE ||
	    buf.buf_len < sizeof(struct tee_ioctl_invoke_arg))
		return -EINVAL;

	uarg = u64_to_user_ptr(buf.buf_ptr);
	if (copy_from_user(&arg, uarg, sizeof(arg)))
		return -EFAULT;

	if (arg.num_params > MAX_NUM_PARAMS)
		return -EINVAL;

	if (sizeof(arg) + TEE_IOCTL_PARAM_SIZE(arg.num_params) != buf.buf_len)
		return -EINVAL;

	if (arg.num_params) {
		params = kcalloc(arg.num_params, sizeof(struct tee_param),
				 GFP_KERNEL);
		if (!params)
			return -ENOMEM;
		uparams = uarg->params;
		rc = params_from_user(ctx, params, (size_t)arg.num_params,
				 uparams);
		if (rc)
			goto out;
	}

	rc = ctx->teedev->desc->ops->invoke_func(ctx, &arg, params);
	if (rc)
		goto out;

	if (put_user(arg.ret, &uarg->ret) ||
	    put_user(arg.ret_origin, &uarg->ret_origin)) {
		rc = -EFAULT;
		goto out;
	}
	rc = params_to_user(uparams, (size_t)arg.num_params, params);
out:
	if (params) {
		/* Decrease ref count for all valid shared memory pointers */
		for (n = 0; n < arg.num_params; n++)
			if (tee_param_is_memref(params + n) &&
			    params[n].u.memref.shm)
				isee_shm_put(params[n].u.memref.shm);
		kfree(params);
	}
	return rc;
}

static int tee_ioctl_cancel(struct tee_context *ctx,
			    struct tee_ioctl_cancel_arg __user *uarg)
{
	struct tee_ioctl_cancel_arg arg;

	if (!ctx->teedev->desc->ops->cancel_req)
		return -EINVAL;

	if (copy_from_user(&arg, uarg, sizeof(arg)))
		return -EFAULT;

	return ctx->teedev->desc->ops->cancel_req(ctx, arg.cancel_id,
						  arg.session);
}

static int
tee_ioctl_close_session(struct tee_context *ctx,
			struct tee_ioctl_close_session_arg __user *uarg)
{
	struct tee_ioctl_close_session_arg arg;

	if (!ctx->teedev->desc->ops->close_session)
		return -EINVAL;

	if (copy_from_user(&arg, uarg, sizeof(arg)))
		return -EFAULT;

	return ctx->teedev->desc->ops->close_session(ctx, arg.session);
}

static int params_to_supp(struct tee_context *ctx,
			  struct tee_ioctl_param __user *uparams,
			  size_t num_params, struct tee_param *params)
{
	size_t n;

	for (n = 0; n < num_params; n++) {
		struct tee_ioctl_param ip;
		struct tee_param *p = params + n;

		ip.attr = p->attr;
		switch (p->attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) {
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT:
			ip.a = p->u.value.a;
			ip.b = p->u.value.b;
			ip.c = p->u.value.c;
			break;
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT:
			ip.b = p->u.memref.size;
			if (!p->u.memref.shm) {
				ip.a = 0;
				ip.c = (u64)-1; /* invalid shm id */
				break;
			}
			ip.a = p->u.memref.shm_offs;
			ip.c = p->u.memref.shm->id;
			break;
		default:
			ip.a = 0;
			ip.b = 0;
			ip.c = 0;
			break;
		}

		if (copy_to_user(uparams + n, &ip, sizeof(ip)))
			return -EFAULT;
	}

	return 0;
}

static int tee_ioctl_supp_recv(struct tee_context *ctx,
			       struct tee_ioctl_buf_data __user *ubuf)
{
	int rc;
	struct tee_ioctl_buf_data buf;
	struct tee_iocl_supp_recv_arg __user *uarg;
	struct tee_param *params;
	u32 num_params;
	u32 func;

	if (!ctx->teedev->desc->ops->supp_recv)
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, sizeof(buf)))
		return -EFAULT;

	if (buf.buf_len > TEE_MAX_ARG_SIZE ||
	    buf.buf_len < sizeof(struct tee_iocl_supp_recv_arg))
		return -EINVAL;

	uarg = u64_to_user_ptr(buf.buf_ptr);
	if (get_user(num_params, &uarg->num_params))
		return -EFAULT;

	if (sizeof(*uarg) + TEE_IOCTL_PARAM_SIZE(num_params) != buf.buf_len)
		return -EINVAL;

	params = kcalloc(num_params, sizeof(struct tee_param), GFP_KERNEL);
	if (!params)
		return -ENOMEM;

	rc = params_from_user(ctx, params, num_params, uarg->params);
	if (rc)
		goto out;

	rc = ctx->teedev->desc->ops->supp_recv(ctx, &func, &num_params, params);
	if (rc)
		goto out;

	if (put_user(func, &uarg->func) ||
	    put_user(num_params, &uarg->num_params)) {
		rc = -EFAULT;
		goto out;
	}

	rc = params_to_supp(ctx, uarg->params, num_params, params);
out:
	kfree(params);
	return rc;
}

static int params_from_supp(struct tee_param *params, size_t num_params,
			    struct tee_ioctl_param __user *uparams)
{
	size_t n;

	for (n = 0; n < num_params; n++) {
		struct tee_param *p = params + n;
		struct tee_ioctl_param ip;

		if (copy_from_user(&ip, uparams + n, sizeof(ip)))
			return -EFAULT;

		/* All unused attribute bits has to be zero */
		if (ip.attr & ~TEE_IOCTL_PARAM_ATTR_MASK)
			return -EINVAL;

		p->attr = ip.attr;
		switch (ip.attr & TEE_IOCTL_PARAM_ATTR_TYPE_MASK) {
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT:
			/* Only out and in/out values can be updated */
			p->u.value.a = ip.a;
			p->u.value.b = ip.b;
			p->u.value.c = ip.c;
			break;
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_OUTPUT:
		case TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT:
			/*
			 * Only the size of the memref can be updated.
			 * Since we don't have access to the original
			 * parameters here, only store the supplied size.
			 * The driver will copy the updated size into the
			 * original parameters.
			 */
			p->u.memref.shm = NULL;
			p->u.memref.shm_offs = 0;
			p->u.memref.size = ip.b;
			break;
		default:
			memset(&p->u, 0, sizeof(p->u));
			break;
		}
	}
	return 0;
}

static int tee_ioctl_supp_send(struct tee_context *ctx,
			       struct tee_ioctl_buf_data __user *ubuf)
{
	long rc;
	struct tee_ioctl_buf_data buf;
	struct tee_iocl_supp_send_arg __user *uarg;
	struct tee_param *params;
	u32 num_params;
	u32 ret;

	/* Not valid for this driver */
	if (!ctx->teedev->desc->ops->supp_send)
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, sizeof(buf)))
		return -EFAULT;

	if (buf.buf_len > TEE_MAX_ARG_SIZE ||
	    buf.buf_len < sizeof(struct tee_iocl_supp_send_arg))
		return -EINVAL;

	uarg = u64_to_user_ptr(buf.buf_ptr);
	if (get_user(ret, &uarg->ret) ||
	    get_user(num_params, &uarg->num_params))
		return -EFAULT;

	if (sizeof(*uarg) + TEE_IOCTL_PARAM_SIZE(num_params) > buf.buf_len)
		return -EINVAL;

	params = kcalloc(num_params, sizeof(struct tee_param), GFP_KERNEL);
	if (!params)
		return -ENOMEM;

	rc = params_from_supp(params, num_params, uarg->params);
	if (rc)
		goto out;

	rc = ctx->teedev->desc->ops->supp_send(ctx, ret, num_params, params);
out:
	kfree(params);
	return rc;
}

static int tee_ioctl_set_hostname(struct tee_context *ctx,
			struct tee_ioctl_close_session_arg __user *uarg)
{
	struct tee_ioctl_set_hostname_arg arg;

	if (copy_from_user(&arg, uarg, sizeof(arg)))
		return -EFAULT;

	memcpy(ctx->hostname, arg.hostname, TEE_MAX_HOSTNAME_SIZE-1);

	return 0;
}

static int tee_ioctl_shm_id(struct tee_context *ctx, unsigned long uaddr)
{
	struct tee_device *teedev = ctx->teedev;
	struct tee_shm *shm = NULL;
	int shm_found = 0;

	mutex_lock(&teedev->mutex);

	list_for_each_entry(shm, &(ctx->list_shm), link) {
		if (shm->uaddr == uaddr) {
			shm_found = 1;
			break;
		}
	}

	mutex_unlock(&teedev->mutex);

	if (shm_found == 0) {
		IMSG_ERROR("Failed to find the shm with uaddr = %llx\n", uaddr);
		return -EINVAL;
	}

	return shm->id;
}

static int tee_ioctl_shm_release(struct tee_context *ctx, unsigned long arg)
{
	int shm_id = 0;
	struct tee_shm *shm = NULL;

	shm_id = (int)arg;

	shm = isee_shm_get_from_id(ctx, shm_id);
	if (IS_ERR(shm)) {
		IMSG_ERROR("Failed to the shm with ID %d\n", shm_id);
		return PTR_ERR(shm);
	}

	isee_shm_kfree(shm);

	return 0;
}

long tee_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct tee_context *ctx = filp->private_data;
	void __user *uarg = (void __user *)arg;
	long retVal = 0;

#if IS_ENABLED(CONFIG_MICROTRUST_TEST_DRIVERS)
	if (cmd != TEE_IOC_CAPI_PROXY)
#endif
		mutex_lock(&ctx->mutex);

	switch (cmd) {
	case TEE_IOC_VERSION:
		retVal = tee_ioctl_version(ctx, uarg);
		break;
	case TEE_IOC_SHM_RELEASE:
		retVal = tee_ioctl_shm_release(ctx, arg);
		break;
	case TEE_IOC_SHM_ID:
		retVal = tee_ioctl_shm_id(ctx, arg);
		break;
#if IS_ENABLED(CONFIG_MICROTRUST_TEST_DRIVERS)
	case TEE_IOC_SHM_KERN_OP:
		retVal = tee_ioctl_shm_kern_op(ctx, uarg);
		break;
	case TEE_IOC_CAPI_PROXY:
		retVal = tee_ioctl_capi_proxy(ctx, uarg);
		break;
#endif
	case TEE_IOC_OPEN_SESSION:
		retVal = tee_ioctl_open_session(ctx, uarg);
		break;
	case TEE_IOC_INVOKE:
		retVal = tee_ioctl_invoke(ctx, uarg);
		break;
	case TEE_IOC_CANCEL:
		retVal = tee_ioctl_cancel(ctx, uarg);
		break;
	case TEE_IOC_CLOSE_SESSION:
		retVal = tee_ioctl_close_session(ctx, uarg);
		break;
	case TEE_IOC_SUPPL_RECV:
		retVal = tee_ioctl_supp_recv(ctx, uarg);
		break;
	case TEE_IOC_SUPPL_SEND:
		retVal = tee_ioctl_supp_send(ctx, uarg);
		break;
	case TEE_IOC_SET_HOSTNAME:
		retVal = tee_ioctl_set_hostname(ctx, uarg);
		break;
	default:
		retVal = -EINVAL;
	}

#if IS_ENABLED(CONFIG_MICROTRUST_TEST_DRIVERS)
	if (cmd != TEE_IOC_CAPI_PROXY)
#endif
		mutex_unlock(&ctx->mutex);

	return retVal;
}

static int tee_mmap(struct file *filp, struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;
	struct tee_context *ctx = filp->private_data;
	struct tee_shm *shm = NULL;
	int retVal = 0;

	mutex_lock(&ctx->mutex);

	shm = isee_shm_kalloc(ctx, size, TEE_SHM_MAPPED | TEE_SHM_DMA_KERN_BUF);
	if (IS_ERR(shm)) {
		IMSG_ERROR("Failed to alloc shm %ld\n", PTR_ERR(shm));
		retVal = PTR_ERR(shm);
		goto exit;
	}

	retVal = remap_pfn_range(vma, vma->vm_start, shm->paddr >> PAGE_SHIFT,
				size, vma->vm_page_prot);

	if (retVal != 0) {
		IMSG_ERROR("Failed to remap the shm %d\n", retVal);
		isee_shm_kfree(shm);
	} else
		shm->uaddr = vma->vm_start;
exit:
	mutex_unlock(&ctx->mutex);

	return retVal;
}


static const struct file_operations tee_fops = {
	.owner = THIS_MODULE,
	.open = tee_open,
	.release = tee_release,
	.unlocked_ioctl = tee_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tee_ioctl,
#endif
	.mmap = tee_mmap,
};

static void tee_release_device(struct device *dev)
{
	struct tee_device *teedev = container_of(dev, struct tee_device, dev);

	spin_lock(&driver_lock);
	clear_bit(teedev->id, dev_mask);
	spin_unlock(&driver_lock);
	mutex_destroy(&teedev->mutex);
	idr_destroy(&teedev->idr);
	kfree(teedev);
}

/**
 * isee_device_alloc() - Allocate a new struct tee_device instance
 * @teedesc:	Descriptor for this driver
 * @dev:	Parent device for this device
 * @pool:	Shared memory pool, NULL if not used
 * @driver_data: Private driver data for this device
 *
 * Allocates a new struct tee_device instance. The device is
 * removed by isee_device_unregister().
 *
 * @returns a pointer to a 'struct tee_device' or an ERR_PTR on failure
 */
struct tee_device *isee_device_alloc(const struct tee_desc *teedesc,
				    struct device *dev,
				    struct tee_shm_pool *pool,
				    void *driver_data)
{
	struct tee_device *teedev;
	void *ret;
	int rc;
	int offs = 0;

	if (!teedesc || !teedesc->name || !teedesc->ops ||
	    !teedesc->ops->get_version || !teedesc->ops->open ||
	    !teedesc->ops->release || !pool)
		return ERR_PTR(-EINVAL);

	teedev = kzalloc(sizeof(*teedev), GFP_KERNEL);
	if (!teedev) {
		ret = ERR_PTR(-ENOMEM);
		goto err;
	}

	if (teedesc->flags & TEE_DESC_PRIVILEGED)
		offs = TEE_NUM_DEVICES / 2;

	spin_lock(&driver_lock);
	teedev->id = find_next_zero_bit(dev_mask, TEE_NUM_DEVICES, offs);
	if (teedev->id < TEE_NUM_DEVICES)
		set_bit(teedev->id, dev_mask);
	spin_unlock(&driver_lock);

	if (teedev->id >= TEE_NUM_DEVICES) {
		ret = ERR_PTR(-ENOMEM);
		goto err;
	}

	rc = snprintf(teedev->name, sizeof(teedev->name), "isee_tee%s%d",
		 teedesc->flags & TEE_DESC_PRIVILEGED ? "priv" : "",
		 teedev->id - offs);
	if (rc <= 0) {
		IMSG_ERROR("failed to call snprintf rc = %d\n", rc);
		ret = ERR_PTR(-EINVAL);
		goto err;
	}

	teedev->dev.class = tee_class;
	teedev->dev.release = tee_release_device;
	teedev->dev.parent = dev;

	teedev->dev.devt = MKDEV(MAJOR(tee_devt), teedev->id);

	rc = dev_set_name(&teedev->dev, "%s", teedev->name);
	if (rc) {
		ret = ERR_PTR(rc);
		goto err_devt;
	}

	cdev_init(&teedev->cdev, &tee_fops);
	teedev->cdev.owner = teedesc->owner;
	teedev->cdev.kobj.parent = &teedev->dev.kobj;

	dev_set_drvdata(&teedev->dev, driver_data);
	device_initialize(&teedev->dev);

	/* 1 as isee_device_unregister() does one final isee_device_put() */
	teedev->num_users = 1;
	init_completion(&teedev->c_no_users);
	mutex_init(&teedev->mutex);
	idr_init(&teedev->idr);

	teedev->desc = teedesc;
	teedev->pool = pool;

	return teedev;
err_devt:
	unregister_chrdev_region(teedev->dev.devt, 1);
err:
	IMSG_ERROR("could not register %s driver\n",
	       teedesc->flags & TEE_DESC_PRIVILEGED ? "privileged" : "client");
	if (teedev && teedev->id < TEE_NUM_DEVICES) {
		spin_lock(&driver_lock);
		clear_bit(teedev->id, dev_mask);
		spin_unlock(&driver_lock);
	}
	kfree(teedev);
	return ret;
}
EXPORT_SYMBOL_GPL(isee_device_alloc);

static ssize_t implementation_id_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct tee_device *teedev = container_of(dev, struct tee_device, dev);
	struct tee_ioctl_version_data vers;

	teedev->desc->ops->get_version(teedev, &vers);
	return scnprintf(buf, PAGE_SIZE, "%d\n", vers.impl_id);
}
static DEVICE_ATTR_RO(implementation_id);

static struct attribute *tee_dev_attrs[] = {
	&dev_attr_implementation_id.attr,
	NULL
};

static const struct attribute_group tee_dev_group = {
	.attrs = tee_dev_attrs,
};

/**
 * isee_device_register() - Registers a TEE device
 * @teedev:	Device to register
 *
 * isee_device_unregister() need to be called to remove the @teedev if
 * this function fails.
 *
 * @returns < 0 on failure
 */
int isee_device_register(struct tee_device *teedev)
{
	int rc;

	if (teedev->flags & TEE_DEVICE_FLAG_REGISTERED) {
		IMSG_ERROR("attempt to register twice\n");
		return -EINVAL;
	}

	rc = cdev_add(&teedev->cdev, teedev->dev.devt, 1);
	if (rc) {
		IMSG_ERROR(
			"unable to cdev_add() %s, major %d, minor %d, err=%d\n",
			teedev->name, MAJOR(teedev->dev.devt),
			MINOR(teedev->dev.devt), rc);
		return rc;
	}

	rc = device_add(&teedev->dev);
	if (rc) {
		IMSG_ERROR(
			"unable to device_add() %s, major %d, minor %d, err=%d\n",
			teedev->name, MAJOR(teedev->dev.devt),
			MINOR(teedev->dev.devt), rc);
		goto err_device_add;
	}

	rc = sysfs_create_group(&teedev->dev.kobj, &tee_dev_group);
	if (rc) {
		IMSG_ERROR(
			"failed to create sysfs attributes, err=%d\n", rc);
		goto err_sysfs_create_group;
	}

	teedev->flags |= TEE_DEVICE_FLAG_REGISTERED;
	return 0;

err_sysfs_create_group:
	device_del(&teedev->dev);
err_device_add:
	cdev_del(&teedev->cdev);
	return rc;
}
EXPORT_SYMBOL_GPL(isee_device_register);

void isee_device_put(struct tee_device *teedev)
{
	mutex_lock(&teedev->mutex);
	/* Shouldn't put in this state */
	if (!WARN_ON(!teedev->desc)) {
		teedev->num_users--;
		if (!teedev->num_users) {
			teedev->desc = NULL;
			complete(&teedev->c_no_users);
		}
	}
	mutex_unlock(&teedev->mutex);
}

bool isee_device_get(struct tee_device *teedev)
{
	mutex_lock(&teedev->mutex);
	if (!teedev->desc) {
		mutex_unlock(&teedev->mutex);
		return false;
	}
	teedev->num_users++;
	mutex_unlock(&teedev->mutex);
	return true;
}

/**
 * isee_device_unregister() - Removes a TEE device
 * @teedev:	Device to unregister
 *
 * This function should be called to remove the @teedev even if
 * isee_device_register() hasn't been called yet. Does nothing if
 * @teedev is NULL.
 */
void isee_device_unregister(struct tee_device *teedev)
{
	if (!teedev)
		return;

	if (teedev->flags & TEE_DEVICE_FLAG_REGISTERED) {
		sysfs_remove_group(&teedev->dev.kobj, &tee_dev_group);
		cdev_del(&teedev->cdev);
		device_del(&teedev->dev);
	}

	isee_device_put(teedev);
	wait_for_completion(&teedev->c_no_users);

	/*
	 * No need to take a mutex any longer now since teedev->desc was
	 * set to NULL before teedev->c_no_users was completed.
	 */

	teedev->pool = NULL;

	put_device(&teedev->dev);
}
EXPORT_SYMBOL_GPL(isee_device_unregister);

/**
 * isee_get_drvdata() - Return driver_data pointer
 * @teedev:	Device containing the driver_data pointer
 * @returns the driver_data pointer supplied to tee_register().
 */
void *isee_get_drvdata(struct tee_device *teedev)
{
	return dev_get_drvdata(&teedev->dev);
}
EXPORT_SYMBOL_GPL(isee_get_drvdata);

struct match_dev_data {
	struct tee_ioctl_version_data *vers;
	const void *data;
	int (*match)(struct tee_ioctl_version_data *vars, const void *data);
};

static int match_dev(struct device *dev, const void *data)
{
	const struct match_dev_data *match_data = data;
	struct tee_device *teedev = container_of(dev, struct tee_device, dev);

	teedev->desc->ops->get_version(teedev, match_data->vers);
	return match_data->match(match_data->vers, match_data->data);
}

struct tee_context *isee_client_open_context(struct tee_context *start,
			int (*match)(struct tee_ioctl_version_data *,
				const void *),
			const void *data, struct tee_ioctl_version_data *vers)
{
	struct device *dev = NULL;
	struct device *put_dev = NULL;
	struct tee_context *ctx = NULL;
	struct tee_ioctl_version_data v;
	struct match_dev_data match_data = { vers ? vers : &v, data, match };

	if (start)
		dev = &start->teedev->dev;

	do {
		dev = class_find_device(tee_class, dev, &match_data, match_dev);
		if (!dev) {
			ctx = ERR_PTR(-ENOENT);
			break;
		}

		put_device(put_dev);
		put_dev = dev;

		ctx = teedev_open(container_of(dev, struct tee_device, dev));
	} while (IS_ERR(ctx) && PTR_ERR(ctx) != -ENOMEM);

	put_device(put_dev);
	return ctx;
}
EXPORT_SYMBOL_GPL(isee_client_open_context);

void isee_client_close_context(struct tee_context *ctx)
{
	teedev_close_context(ctx);
}
EXPORT_SYMBOL_GPL(isee_client_close_context);

void isee_client_get_version(struct tee_context *ctx,
			struct tee_ioctl_version_data *vers)
{
	ctx->teedev->desc->ops->get_version(ctx->teedev, vers);
}
EXPORT_SYMBOL_GPL(isee_client_get_version);


int isee_client_open_session(struct tee_context *ctx,
			struct tee_ioctl_open_session_arg *arg,
			struct tee_param *param)
{
	if (!ctx->teedev->desc->ops->open_session)
		return -EINVAL;
	return ctx->teedev->desc->ops->open_session(ctx, arg, param);
}
EXPORT_SYMBOL_GPL(isee_client_open_session);

int isee_client_close_session(struct tee_context *ctx, u32 session)
{
	if (!ctx->teedev->desc->ops->close_session)
		return -EINVAL;
	return ctx->teedev->desc->ops->close_session(ctx, session);
}
EXPORT_SYMBOL_GPL(isee_client_close_session);

int isee_client_invoke_func(struct tee_context *ctx,
			struct tee_ioctl_invoke_arg *arg,
			struct tee_param *param)
{
	if (!ctx->teedev->desc->ops->invoke_func)
		return -EINVAL;
	return ctx->teedev->desc->ops->invoke_func(ctx, arg, param);
}
EXPORT_SYMBOL_GPL(isee_client_invoke_func);

int teei_tee_init(void)
{
	int rc;

	tee_class = class_create(THIS_MODULE, "isee_tee");
	if (IS_ERR(tee_class)) {
		IMSG_ERROR("couldn't create class\n");
		return PTR_ERR(tee_class);
	}

	rc = alloc_chrdev_region(&tee_devt, 0, TEE_NUM_DEVICES, "isee_tee");
	if (rc) {
		IMSG_ERROR("failed to allocate char dev region\n");
		class_destroy(tee_class);
		tee_class = NULL;
	}

	return rc;
}
EXPORT_SYMBOL_GPL(teei_tee_init);

void teei_tee_exit(void)
{
	class_destroy(tee_class);
	tee_class = NULL;
	unregister_chrdev_region(tee_devt, TEE_NUM_DEVICES);
}
EXPORT_SYMBOL_GPL(teei_tee_exit);
