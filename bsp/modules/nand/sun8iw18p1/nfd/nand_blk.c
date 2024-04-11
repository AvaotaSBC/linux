/*
 * nand_blk.c for  SUNXI NAND .
 *
 * Copyright (C) 2016 Allwinner.
 *
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
//#include <linux/compat.h>
//#include <linux/uaccess.h>
#include <linux/version.h>
#include "nand_blk.h"
#include "nand_dev.h"

/*****************************************************************************/

#define REMAIN_SPACE 0
#define PART_FREE 0x55
#define PART_DUMMY 0xff
#define PART_READONLY 0x85
#define PART_WRITEONLY 0x86
#define PART_NO_ACCESS 0x87
/* ms*/
#define TIMEOUT				300
#define NAND_CACHE_RW

#define NAND_IO_RESPONSE_TEST
static unsigned int dragonboard_test_flag;
struct secblc_op_t {
	int item;
	unsigned char *buf;
	unsigned int len;
};

struct burn_param_t {
	void *buffer;
	long length;
};

#ifdef CONFIG_COMPAT

struct secblc_op_t32 {
	int item;
	compat_uptr_t buf;
	unsigned int len;
};

struct burn_param_t32 {
	compat_uptr_t buffer;
	compat_size_t length;
};

struct hd_geometry32 {
	unsigned char heads;
	unsigned char sectors;
	unsigned short cylinders;
	compat_size_t start;
};

#endif

DEFINE_SEMAPHORE(nand_mutex);

unsigned char IS_IDLE = 1;
static int nand_ioctl(struct block_device *bdev, fmode_t mode, unsigned int cmd,
		      unsigned long arg);
#ifdef CONFIG_COMPAT
static int nand_compat_ioctl(struct block_device *bdev, fmode_t mode,
			     unsigned int cmd, unsigned long arg);
#endif
long max_r_io_response = 1;
long max_w_io_response = 1;

int debug_data;

#if 0//#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0))
struct timeval tpstart, tpend;
#else
//TODO
#endif
long timeuse;

extern void mark_to_disable_crc_when_ota(void);

/* print flags by name */
/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
void start_time(int data)
{
#ifdef NAND_IO_RESPONSE_TEST

	if (debug_data != data)
		return;
#if 0//#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0))
	do_gettimeofday(&tpstart);
#else
	//TODO
#endif

#endif
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
int end_time(int data, int time, int par)
{
#ifdef NAND_IO_RESPONSE_TEST

	if (debug_data != data)
		return -1;

#if 0//#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0))
	do_gettimeofday(&tpend);
	timeuse =
	    1000 * (tpend.tv_sec - tpstart.tv_sec) * 1000 + (tpend.tv_usec -
							     tpstart.tv_usec);
	if (timeuse > time) {
		nand_dbg_err("%ld %d\n", timeuse, par);
		return 1;
	}
#else
	//TODO
#endif

#endif
	return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int do_blktrans_request(struct nand_blk_ops *tr,
			       struct nand_blk_dev *dev, struct request *req)
{
	int ret = 0;
#if 0//#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0))
	unsigned int block, nsect;
	char *buf;
	struct _nand_dev *nand_dev;

	nand_dev = (struct _nand_dev *)dev->priv;

	block = blk_rq_pos(req) << 9 >> tr->blkshift;
	nsect = blk_rq_cur_bytes(req) >> tr->blkshift;

	buf = bio_data(req->bio);

	if (req->cmd_type != REQ_TYPE_FS) {
		nand_dbg_err(KERN_NOTICE "not type fs\n");
		return -EIO;
	}

	if (req_op(req) == REQ_OP_FLUSH)
		return nand_dev->flush_write_cache(nand_dev, 0xffff);

	if (blk_rq_pos(req) + blk_rq_cur_sectors(req) >
			get_capacity(req->rq_disk)) {
		nand_dbg_err(KERN_NOTICE "over capacity\n");
		return -EIO;
	}

	if (req_op(req) == REQ_OP_DISCARD) {
		/*nand_dev->discard(nand_dev, block, nsect);*/
		goto request_exit;
	}

	switch (rq_data_dir(req)) {
	case READ:
		nand_dev->read_data(nand_dev, block, nsect, buf);
		rq_flush_dcache_pages(req);
		ret = 0;
		goto request_exit;

	case WRITE:
		rq_flush_dcache_pages(req);
		nand_dev->write_data(nand_dev, block, nsect, buf);
		ret = 0;
		goto request_exit;
	default:
		nand_dbg_err("Unknown request %u\n", rq_data_dir(req));
		ret = -EIO;
		goto request_exit;
	}

request_exit:
#else
	//TODO
	unsigned int block, nsect;
	char *buf;
	struct _nand_dev *nand_dev;

	nand_dev = (struct _nand_dev *)dev->priv;

	block = blk_rq_pos(req) << 9 >> tr->blkshift;
	nsect = blk_rq_cur_bytes(req) >> tr->blkshift;

	//buf = bio_data(req->bio);

	//if (req->cmd_type != REQ_TYPE_FS) {
	//	nand_dbg_err(KERN_NOTICE "not type fs\n");
	//	return -EIO;
	//}

	if (req_op(req) == REQ_OP_FLUSH)
		return nand_dev->flush_write_cache(nand_dev, 0xffff);

	if (blk_rq_pos(req) + blk_rq_cur_sectors(req) >
			get_capacity(req->rq_disk)) {
		nand_dbg_err(KERN_NOTICE "over capacity\n");
		return -EIO;
	}

	if (req_op(req) == REQ_OP_DISCARD) {
		/*nand_dev->discard(nand_dev, block, nsect);*/
		goto request_exit;
	}

#if 0
	switch (rq_data_dir(req)) {
	case READ:
		nand_dev->read_data(nand_dev, block, nsect, buf);
		rq_flush_dcache_pages(req);
		ret = 0;
		goto request_exit;

	case WRITE:
		rq_flush_dcache_pages(req);
		nand_dev->write_data(nand_dev, block, nsect, buf);
		ret = 0;
		goto request_exit;
	default:
		nand_dbg_err("Unknown request %u\n", rq_data_dir(req));
		ret = -EIO;
		goto request_exit;
	}
#else
	switch (req_op(req)) {
	case REQ_OP_READ:
		buf = kmap(bio_page(req->bio)) + bio_offset(req->bio);
		nand_dev->read_data(nand_dev, block, nsect, buf);
		kunmap(bio_page(req->bio));
		rq_flush_dcache_pages(req);
		ret = 0;
		goto request_exit;

	case REQ_OP_WRITE:
		rq_flush_dcache_pages(req);
		buf = kmap(bio_page(req->bio)) + bio_offset(req->bio);
		nand_dev->write_data(nand_dev, block, nsect, buf);
		kunmap(bio_page(req->bio));
		ret = 0;

		goto request_exit;
	default:
		nand_dbg_err("Unknown request %u\n", rq_data_dir(req));
		ret = -EIO;
		goto request_exit;
	}
#endif
request_exit:
#endif
	return ret;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int mtd_blktrans_thread(void *arg)
{
#if 0//#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0))
	struct nand_blk_ops *tr = arg;
	struct request_queue *rq = tr->rq;
	struct request *req = NULL;
	struct nand_blk_dev *dev;
	int background_done = 0;

	spin_lock_irq(rq->queue_lock);

	while (!kthread_should_stop()) {
		int res;

		tr->bg_stop = false;
		if (!req) {
			req = blk_fetch_request(rq);
			if (!req) {
				set_current_state(TASK_INTERRUPTIBLE);
				if (kthread_should_stop())
					set_current_state(TASK_RUNNING);

				spin_unlock_irq(rq->queue_lock);
				tr->rq_null++;
				schedule();
				spin_lock_irq(rq->queue_lock);
				continue;
			}
		}

		dev = req->rq_disk->private_data;

		spin_unlock_irq(rq->queue_lock);
		tr->rq_null = 0;
		mutex_lock(&dev->lock);
		res = do_blktrans_request(tr, dev, req);
		mutex_unlock(&dev->lock);

		spin_lock_irq(rq->queue_lock);

		if (!__blk_end_request_cur(req, res))
			req = NULL;

		background_done = 0;
	}

	if (req)
		__blk_end_request_all(req, -EIO);

	spin_unlock_irq(rq->queue_lock);
#else
	//TODO
#define NAND_SCHEDULE_TIMEOUT (HZ >> 1)
	unsigned long time_val = NAND_SCHEDULE_TIMEOUT;
	while (!kthread_should_stop()) {
		printk("<%s:%d>\n", __func__, __LINE__);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(time_val);
	}
#endif
	return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static void mtd_blktrans_request(struct request_queue *rq)
{
#if 0//#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0))
	struct nand_blk_ops *nandr;
	struct request *req = NULL;

	nandr = rq->queuedata;

	if (!nandr)
		while ((req = blk_fetch_request(rq)) != NULL)
			__blk_end_request_all(req, -ENODEV);
	else {
		nandr->bg_stop = true;
		wake_up_process(nandr->thread);
	}
#else
	//TODO
#endif
}

static void null_for_dragonboard(struct request_queue *rq)
{

}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int nand_open(struct block_device *bdev, fmode_t mode)
{
	struct nand_blk_dev *dev;
	struct nand_blk_ops *nandr;
	int ret = -ENODEV;

	//printk("<%s:%d>\n", __func__, __LINE__);

	dev = bdev->bd_disk->private_data;
	nandr = dev->nandr;

	set_bit(GD_NEED_PART_SCAN, &bdev->bd_disk->state);

	if (!try_module_get(nandr->owner)) {
		printk("<%s:%d>\n", __func__, __LINE__);
		goto out;
	}

	ret = 0;
	if (nandr->open) {
		ret = nandr->open(dev);
		//printk("<%s:%d> ret:%d\n", __func__, __LINE__, ret);
		if (ret)
out:
			module_put(nandr->owner);
	}
	return ret;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static void nand_release(struct gendisk *disk, fmode_t mode)
{
	struct nand_blk_dev *dev;
	struct nand_blk_ops *nandr;
	int ret = 0;

	dev = disk->private_data;
	nandr = dev->nandr;
	if (nandr->release)
		ret = nandr->release(dev);

	if (!ret)
		module_put(nandr->owner);
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
#define DISABLE_WRITE           _IO('V', 0)
#define ENABLE_WRITE            _IO('V', 1)
#define DISABLE_READ            _IO('V', 2)
#define ENABLE_READ             _IO('V', 3)
#define DRAGON_BOARD_TEST       _IO('V', 55)
#define BLKREADBOOT0            _IO('v', 125)
#define BLKREADBOOT1            _IO('v', 126)
#define BLKBURNBOOT0            _IO('v', 127)
#define BLKBURNBOOT1            _IO('v', 128)
#define DISABLE_CRC             _IO('v', 129)
#define SECBLK_READ				_IO('V', 20)
#define SECBLK_WRITE			_IO('V', 21)
#define SECBLK_IOCTL			_IO('V', 22)

static int nand_ioctl(struct block_device *bdev, fmode_t mode, unsigned int cmd,
		      unsigned long arg)
{
	struct nand_blk_dev *dev = bdev->bd_disk->private_data;
	struct nand_blk_ops *nandr = dev->nandr;
	struct burn_param_t burn_param;
	struct secblc_op_t sec_op_st;
	int ret = 0;
	unsigned char *buf_secure = NULL;

	switch (cmd) {
	case BLKFLSBUF:
		if (nandr->flush)
			return nandr->flush(dev);
		/* The core code did the work, we had nothing to do. */
		return 0;

	case HDIO_GETGEO:
		if (nandr->getgeo) {
			struct hd_geometry g;
			int ret;

			memset(&g, 0, sizeof(g));
			ret = nandr->getgeo(dev, &g);
			if (ret)
				return ret;
			nand_dbg_err("HDIO_GETGEO called!\n");
			g.start = get_start_sect(bdev);
			if (copy_to_user((void __user *)arg, &g, sizeof(g)))
				return -EFAULT;

			return 0;
		}
		return 0;
	case ENABLE_WRITE:
		nand_dbg_err("enable write!\n");
		dev->disable_access = 0;
		dev->readonly = 0;
		set_disk_ro(dev->disk, 0);
		return 0;

	case DISABLE_WRITE:
		nand_dbg_err("disable write!\n");
		dev->readonly = 1;
		set_disk_ro(dev->disk, 1);
		return 0;

	case ENABLE_READ:
		nand_dbg_err("enable read!\n");
		dev->disable_access = 0;
		dev->writeonly = 0;
		return 0;

	case DISABLE_READ:
		nand_dbg_err("disable read!\n");
		dev->writeonly = 1;
		return 0;

	case BLKREADBOOT0:
		nand_dbg_err("start BLKREADBOOT0...\n");
		if (copy_from_user(&burn_param,
			(struct burn_param_t __user *)arg,
			sizeof(burn_param))) {
			nand_dbg_err("nand_ioctl input arg err\n");
			return -EINVAL;
		}
		down(&nandr->nand_ops_mutex);
		IS_IDLE = 0;
		ret = NAND_ReadBoot0(burn_param.length, burn_param.buffer);
		if (ret != 0)
			nand_dbg_err("BLKREADBOOT0 failed\n");
		up(&(nandr->nand_ops_mutex));
		nand_dbg_err("do BLKREADBOOT0!\n");
		IS_IDLE = 1;
		return ret;

	case BLKREADBOOT1:
		nand_dbg_err("start BLKREADBOOT1...\n");
		if (copy_from_user(&burn_param,
			(struct burn_param_t __user *)arg,
			sizeof(burn_param))) {
			nand_dbg_err("nand_ioctl input arg err\n");
			return -EINVAL;
		}

		down(&nandr->nand_ops_mutex);
		IS_IDLE = 0;
		ret = NAND_ReadBoot1(burn_param.length, burn_param.buffer);
		if (ret != 0)
			nand_dbg_err("BLKREADBOOT1 failed\n");
		up(&(nandr->nand_ops_mutex));
		nand_dbg_err("do BLKREADBOOT1!\n");
		IS_IDLE = 1;
		return ret;

	case BLKBURNBOOT0:
		nand_dbg_err("start BLKBURNBOOT0...\n");
		if (copy_from_user(&burn_param,
			(struct burn_param_t __user *)arg,
			sizeof(burn_param))) {
			nand_dbg_err("nand_ioctl input arg err\n");
			return -EINVAL;
		}

		down(&nandr->nand_ops_mutex);
		IS_IDLE = 0;
		ret = NAND_BurnBoot0(burn_param.length, burn_param.buffer);
		if (ret != 0)
			nand_dbg_err("BLKBURNBOOT0 failed\n");
		up(&(nandr->nand_ops_mutex));
		nand_dbg_err("do BLKBURNBOOT0!\n");
		IS_IDLE = 1;
		return ret;

	case BLKBURNBOOT1:
		nand_dbg_err("start BLKBURNBOOT1...\n");
		if (copy_from_user(&burn_param,
			(struct burn_param_t __user *)arg,
			sizeof(burn_param))) {
			nand_dbg_err("nand_ioctl input arg err\n");
			return -EINVAL;
		}
		down(&nandr->nand_ops_mutex);
		IS_IDLE = 0;
		ret = NAND_BurnBoot1(burn_param.length, burn_param.buffer);
		if (ret != 0)
			nand_dbg_err("BLKBURNBOOT1 failed\n");
		up(&(nandr->nand_ops_mutex));
		nand_dbg_err("do BLKBURNBOOT1!\n");
		IS_IDLE = 1;
		return ret;

	case DISABLE_CRC:
		nand_dbg_err("mark to disable crc when ota!\n");
		mark_to_disable_crc_when_ota();
		return 0;

	case SECBLK_READ:

		nand_dbg_err("start secure read ...\n");
		down(&nandr->nand_ops_mutex);
		IS_IDLE = 0;
		if (copy_from_user(&sec_op_st,
			(struct secblc_op_t __user *)arg,
			sizeof(sec_op_st))) {
			nand_dbg_err("nand_ioctl input arg err\n");
			return -EINVAL;
		}
		buf_secure = kmalloc(sec_op_st.len, GFP_KERNEL);
		if (buf_secure == NULL) {
			nand_dbg_err("buf_secure malloc fail!\n");
			return -1;
		}
		ret =
		    nand_secure_storage_read(sec_op_st.item, buf_secure,
					     sec_op_st.len);
		if (copy_to_user(sec_op_st.buf, buf_secure, sec_op_st.len))
			ret = -EFAULT;
		kfree(buf_secure);
		up(&(nandr->nand_ops_mutex));
		IS_IDLE = 1;
		return ret;

	case SECBLK_WRITE:

		nand_dbg_err("start secure write ...\n");
		down(&nandr->nand_ops_mutex);
		IS_IDLE = 0;
		if (copy_from_user(&sec_op_st,
			(struct secblc_op_t __user *)arg,
			sizeof(sec_op_st))) {
			nand_dbg_err("nand_ioctl input arg err\n");
			return -EINVAL;
		}
		buf_secure = kmalloc(sec_op_st.len, GFP_KERNEL);
		if (buf_secure == NULL) {
			nand_dbg_err("buf_secure malloc fail!\n");
			return -1;
		}
		if (copy_from_user
		    (buf_secure, (const void *)sec_op_st.buf, sec_op_st.len))
			ret = -EFAULT;
		ret =
		    nand_secure_storage_write(sec_op_st.item, buf_secure,
					      sec_op_st.len);
		kfree(buf_secure);
		up(&(nandr->nand_ops_mutex));
		IS_IDLE = 1;
		return ret;

	case SECBLK_IOCTL:

		nand_dbg_err("start secure get item...\n");
		down(&nandr->nand_ops_mutex);
		IS_IDLE = 0;
		ret = get_nand_secure_storage_max_item();
		up(&(nandr->nand_ops_mutex));
		IS_IDLE = 1;
		return ret;

	case DRAGON_BOARD_TEST:

		nand_dbg_err("start dragonborad test...\n");
		down(&(nandr->nand_ops_mutex));
		IS_IDLE = 0;
		ret = NAND_DragonboardTest();
		up(&(nandr->nand_ops_mutex));
		IS_IDLE = 1;
		return ret;

	default:
		nand_dbg_err("unknown cmd 0x%x!\n", cmd);
		return -ENOTTY;
	}
}

#ifdef CONFIG_COMPAT
int nand_readboot_compat(struct block_device *bdev, fmode_t mode,
			  unsigned int cmd, struct burn_param_t32 __user *arg)
{
	struct burn_param_t32 burn_param32;
	struct burn_param_t __user *burn_param;
	int ret;

	if (copy_from_user(&burn_param32, arg, sizeof(burn_param32)))
		return -EFAULT;

	burn_param = compat_alloc_user_space(sizeof(*burn_param));
	if (!access_ok(VERIFY_WRITE, burn_param, sizeof(*burn_param)))
		return -EFAULT;

	if (put_user(compat_ptr(burn_param32.buffer), &burn_param->buffer) ||
	    put_user(burn_param32.length, &burn_param->length))
		return -EFAULT;

	ret = nand_ioctl(bdev, mode, cmd, (unsigned long)burn_param);
	if (ret < 0)
		return ret;

	if (copy_in_user(&arg->buffer, &burn_param->buffer,
			sizeof(arg->buffer)) ||
	    copy_in_user(&arg->length, &burn_param->length,
			sizeof(arg->length)))
		return -EFAULT;

	return 0;
}

int nand_burnboot_compat(struct block_device *bdev, fmode_t mode,
			  unsigned int cmd, struct burn_param_t32 __user *arg)
{
	struct burn_param_t32 burn_param32;
	struct burn_param_t __user *burn_param;
	int ret;

	if (copy_from_user(&burn_param32, arg, sizeof(burn_param32)))
		return -EFAULT;

	burn_param = compat_alloc_user_space(sizeof(*burn_param));
	if (!access_ok(VERIFY_WRITE, burn_param, sizeof(*burn_param)))
		return -EFAULT;

	if (put_user(compat_ptr(burn_param32.buffer), &burn_param->buffer) ||
	    put_user(burn_param32.length, &burn_param->length))
		return -EFAULT;

	ret = nand_ioctl(bdev, mode, cmd, (unsigned long)burn_param);
	if (ret < 0)
		return ret;

	return 0;
}

int nand_drangonboard_compat(struct block_device *bdev, fmode_t mode,
			     unsigned int cmd, unsigned long __user *arg)
{
	int ret;

	ret = nand_ioctl(bdev, mode, cmd, (unsigned long)arg);
	if (ret < 0)
		return ret;

	return 0;
}

int nand_securestorage_compat(struct block_device *bdev, fmode_t mode,
			      unsigned int cmd,
			      struct secblc_op_t32 __user *arg)
{
	struct secblc_op_t32 secblc_op32;
	struct secblc_op_t __user *secblc_op;
	int ret;

	if (copy_from_user(&secblc_op32, arg, sizeof(secblc_op32)))
		return -EFAULT;

	secblc_op = compat_alloc_user_space(sizeof(*secblc_op));
	if (!access_ok(VERIFY_WRITE, secblc_op, sizeof(*secblc_op)))
		return -EFAULT;

	if (put_user(secblc_op32.item, &secblc_op->item) ||
	    put_user(compat_ptr(secblc_op32.buf), &secblc_op->buf) ||
	    put_user(secblc_op32.len, &secblc_op->len))
		return -EFAULT;

	ret = nand_ioctl(bdev, mode, cmd, (unsigned long)secblc_op);
	if (ret < 0)
		return ret;

	if (copy_in_user(&arg->item, &secblc_op->item, sizeof(arg->item)) ||
	    copy_in_user(&arg->len, &secblc_op->len, sizeof(arg->len)))
		return -EFAULT;

	return 0;
}

int nand_getgeo_compat(struct block_device *bdev, fmode_t mode,
		       unsigned int cmd, struct hd_geometry32 __user *arg)
{
	struct hd_geometry32 getgeo32;
	struct hd_geometry __user *getgeo;
	int ret;

	if (copy_from_user(&getgeo32, arg, sizeof(getgeo32)))
		return -EFAULT;

	getgeo = compat_alloc_user_space(sizeof(*getgeo));
	if (!access_ok(VERIFY_WRITE, getgeo, sizeof(*getgeo)))
		return -EFAULT;

	if (put_user(getgeo32.heads, &getgeo->heads) ||
	    put_user(getgeo32.sectors, &getgeo->sectors) ||
	    put_user(getgeo32.cylinders, &getgeo->cylinders) ||
	    put_user(getgeo32.start, &getgeo->start))
		return -EFAULT;

	ret = nand_ioctl(bdev, mode, cmd, (unsigned long)getgeo);
	if (ret < 0)
		return ret;

	if (copy_in_user(&arg->heads, &getgeo->heads, sizeof(arg->heads)) ||
	    copy_in_user(&arg->sectors, &getgeo->sectors, sizeof(arg->sectors))
	    || copy_in_user(&arg->cylinders, &getgeo->cylinders,
			    sizeof(arg->cylinders))
	    || copy_in_user(&arg->start, &getgeo->start, sizeof(arg->start)))
		return -EFAULT;

	return 0;
}

static int nand_compat_ioctl(struct block_device *bdev, fmode_t mode,
			     unsigned int cmd, unsigned long arg)
{
	nand_dbg_err("start nand_compat_ioctl\n");
	switch (cmd) {
	case HDIO_GETGEO:
		return nand_getgeo_compat(bdev, mode, cmd, compat_ptr(arg));

	case BLKREADBOOT0:
	case BLKREADBOOT1:
		return nand_readboot_compat(bdev, mode, cmd, compat_ptr(arg));

	case BLKBURNBOOT0:
	case BLKBURNBOOT1:
		return nand_burnboot_compat(bdev, mode, cmd, compat_ptr(arg));

	case DRAGON_BOARD_TEST:
		return nand_drangonboard_compat(bdev, mode, cmd,
						compat_ptr(arg));

	case SECBLK_READ:
	case SECBLK_WRITE:
		return nand_securestorage_compat(bdev, mode, cmd,
						 compat_ptr(arg));

	default:
		return nand_ioctl(bdev, mode, cmd, arg);
	}
}
#endif

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/

const struct block_device_operations nand_blktrans_ops = {
	.owner = THIS_MODULE,
	.open = nand_open,
	.release = nand_release,
	.ioctl = nand_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nand_compat_ioctl,
#endif

};

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int nand_blk_open(struct nand_blk_dev *dev)
{
#if 0
	nand_dbg_err("nand_blk_open!\n");
	mutex_lock(&dev->lock);
	nand_dbg_err("nand_open ok!\n");

	kref_get(&dev->ref);

	mutex_unlock(&dev->lock);
#endif
	return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int nand_blk_release(struct nand_blk_dev *dev)
{
	int error = 0;
	struct _nand_dev *nand_dev = (struct _nand_dev *)dev->priv;

	if (dragonboard_test_flag == 0) {
		error = nand_dev->flush_write_cache(nand_dev, 0xffff);
		return error;
	} else
		return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
int del_nand_blktrans_dev(struct nand_blk_dev *dev)
{
#if 0
	if (!down_trylock(&nand_mutex)) {
		up(&nand_mutex);
		BUG();
	}
	blk_cleanup_queue(dev->rq);
	kthread_stop(dev->thread);
#endif
	list_del(&dev->list);
	dev->disk->queue = NULL;
	del_gendisk(dev->disk);
	put_disk(dev->disk);

	return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
static int nand_getgeo(struct nand_blk_dev *dev, struct hd_geometry *geo)
{
	geo->heads = dev->heads;
	geo->sectors = dev->sectors;
	geo->cylinders = dev->cylinders;

	return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
struct nand_blk_ops mytr = {
	.name = "nand",
	.major = 93,
	.minorbits = 5,
	.blksize = 512,
	.blkshift = 9,
	.open = nand_blk_open,
	.release = nand_blk_release,
	.getgeo = nand_getgeo,
	.add_dev = add_nand,
	.add_dev_test = add_nand_for_dragonboard_test,
	.remove_dev = remove_nand,
	.flush = nand_flush,
	.owner = THIS_MODULE,
};

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
int add_nand_blktrans_dev(struct nand_blk_dev *dev)
{
#if 0//#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0))
	struct nand_blk_ops *tr = dev->nandr;
	struct list_head *this;
	struct gendisk *gd;
	unsigned long temp;
	int ret = -ENOMEM;

	int last_devnum = -1;

	dev->cylinders = 1024;
	dev->heads = 16;

	temp = dev->cylinders * dev->heads;
	dev->sectors = (dev->size) / temp;
	if ((dev->size) % temp) {
		dev->sectors++;
		temp = dev->cylinders * dev->sectors;
		dev->heads = (dev->size) / temp;

		if ((dev->size) % temp) {
			dev->heads++;
			temp = dev->heads * dev->sectors;
			dev->cylinders = (dev->size) / temp;
		}
	}

	if (!down_trylock(&nand_mutex)) {
		up(&nand_mutex);
		BUG();
	}

	list_for_each(this, &tr->devs) {
		struct nand_blk_dev *tmpdev =
		    list_entry(this, struct nand_blk_dev, list);
		if (dev->devnum == -1) {
			/* Use first free number */
			if (tmpdev->devnum != last_devnum + 1) {
				/* Found a free devnum. Plug it in here */
				dev->devnum = last_devnum + 1;
				list_add_tail(&dev->list, &tmpdev->list);
				goto added;
			}
		} else if (tmpdev->devnum == dev->devnum) {
			/* Required number taken */
			nand_dbg_err("\nerror00\n");
			return -EBUSY;
		} else if (tmpdev->devnum > dev->devnum) {
			/* Required number was free */
			list_add_tail(&dev->list, &tmpdev->list);
			goto added;
		}
		last_devnum = tmpdev->devnum;
	}
	if (dev->devnum == -1)
		dev->devnum = last_devnum + 1;

	if ((dev->devnum << tr->minorbits) > 256) {
		nand_dbg_err("\nerror00000\n");
		return -EBUSY;
	}

	list_add_tail(&dev->list, &tr->devs);

added:
	gd = alloc_disk(1 << tr->minorbits);
	if (!gd) {
		list_del(&dev->list);
		goto error2;
	}

	gd->major = tr->major;
	gd->first_minor = (dev->devnum) << tr->minorbits;
	gd->fops = &nand_blktrans_ops;
	if (NAND_SUPPORT_GPT)
		snprintf(gd->disk_name, sizeof(gd->disk_name), "%s%c", tr->name,
			(tr->minorbits ? '0' : '0') + dev->devnum);
	else
		snprintf(gd->disk_name, sizeof(gd->disk_name), "%s%c", tr->name,
			(tr->minorbits ? 'a' : '0') + dev->devnum);

	set_capacity(gd, dev->size);

	gd->private_data = dev;
	dev->disk = gd;
	tr->rq->bypass_depth++;
	gd->queue = tr->rq;

	dev->disable_access = 0;
	dev->readonly = 0;
	dev->writeonly = 0;
	mutex_init(&dev->lock);
	device_add_disk(ndfc_dev->parent, gd);

	return 0;

error2:
	nand_dbg_err("\nerror2\n");
	list_del(&dev->list);
#else
	//TODO
	struct nand_blk_ops *tr = dev->nandr;
	struct list_head *this;
	struct gendisk *gd;
	unsigned long temp;
	int ret = -ENOMEM;

	int last_devnum = -1;

	printk("<%s:%d> dev: %lx\n", __func__, __LINE__, (unsigned long)dev);

	dev->cylinders = 1024;
	dev->heads = 16;

	temp = dev->cylinders * dev->heads;
	dev->sectors = (dev->size) / temp;
	if ((dev->size) % temp) {
		dev->sectors++;
		temp = dev->cylinders * dev->sectors;
		dev->heads = (dev->size) / temp;

		if ((dev->size) % temp) {
			dev->heads++;
			temp = dev->heads * dev->sectors;
			dev->cylinders = (dev->size) / temp;
		}
	}

	if (!down_trylock(&nand_mutex)) {
		up(&nand_mutex);
		BUG();
	}

	list_for_each(this, &tr->devs) {
		struct nand_blk_dev *tmpdev =
		    list_entry(this, struct nand_blk_dev, list);
		printk("<%s:%d> tmpdev: %lx\n", __func__, __LINE__, (unsigned long)tmpdev);
		if (dev->devnum == -1) {
			/* Use first free number */
			if (tmpdev->devnum != last_devnum + 1) {
				/* Found a free devnum. Plug it in here */
				dev->devnum = last_devnum + 1;
				list_add_tail(&dev->list, &tmpdev->list);
				goto added;
			}
		} else if (tmpdev->devnum == dev->devnum) {
			/* Required number taken */
			nand_dbg_err("\nerror00\n");
			return -EBUSY;
		} else if (tmpdev->devnum > dev->devnum) {
			/* Required number was free */
			list_add_tail(&dev->list, &tmpdev->list);
			goto added;
		}
		last_devnum = tmpdev->devnum;
	}
	if (dev->devnum == -1)
		dev->devnum = last_devnum + 1;

	if ((dev->devnum << tr->minorbits) > 256) {
		nand_dbg_err("\nerror00000\n");
		return -EBUSY;
	}

	list_add_tail(&dev->list, &tr->devs);

added:
	gd = alloc_disk(1 << tr->minorbits);
	printk("<%s:%d> gd: %lx\n", __func__, __LINE__, (unsigned long)gd);
	if (!gd) {
		list_del(&dev->list);
		goto error2;
	}

	gd->major = tr->major;
	gd->first_minor = (dev->devnum) << tr->minorbits;
	gd->fops = &nand_blktrans_ops;
	if (NAND_SUPPORT_GPT)
		snprintf(gd->disk_name, sizeof(gd->disk_name), "%s%c", tr->name,
			(tr->minorbits ? '0' : '0') + dev->devnum);
	else
		snprintf(gd->disk_name, sizeof(gd->disk_name), "%s%c", tr->name,
			(tr->minorbits ? 'a' : '0') + dev->devnum);

	set_capacity(gd, dev->size);

	gd->private_data = dev;
	dev->disk = gd;
	//tr->rq->bypass_depth++;
	gd->queue = tr->rq;

	dev->disable_access = 0;
	dev->readonly = 0;
	dev->writeonly = 0;
	mutex_init(&dev->lock);
	device_add_disk(ndfc_dev->parent, gd, NULL);
	//blk_add_partitions
	return 0;

error2:
	nand_dbg_err("\nerror2\n");
	list_del(&dev->list);
#endif
	return ret;
}

int add_nand_blktrans_dev_for_dragonboard(struct nand_blk_dev *dev)
{
#if 0//#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0))
	struct nand_blk_ops *tr = dev->nandr;
	struct gendisk *gd;
	int ret = -ENOMEM;

	gd = alloc_disk(1);
	if (!gd) {
		list_del(&dev->list);
		goto error2;
	}

	gd->major = tr->major;
	gd->first_minor = 0;
	gd->fops = &nand_blktrans_ops;

	snprintf(gd->disk_name, sizeof(gd->disk_name),
		"%s%c", tr->name, (1 ? 'a' : '0') + dev->devnum);
	set_capacity(gd, 512);

	gd->private_data = dev;
	dev->disk = gd;
	gd->queue = tr->rq;

	dev->disable_access = 0;
	dev->readonly = 0;
	dev->writeonly = 0;

	mutex_init(&dev->lock);

	/* Create the request queue */
	spin_lock_init(&tr->queue_lock);
	tr->rq = blk_init_queue(null_for_dragonboard, &tr->queue_lock);
	if (!tr->rq)
		goto error3;

	tr->rq->queuedata = dev;
	blk_queue_logical_block_size(tr->rq, tr->blksize);

	gd->queue = tr->rq;
	add_disk(gd);

	return 0;

error3:
	nand_dbg_err("\nerror3\n");
	put_disk(dev->disk);
error2:
	nand_dbg_err("\nerror2\n");
	list_del(&dev->list);
#else
	int ret = -ENOMEM;
	//TODO
	printk("<%s:%d> dev: %lx\n", __func__, __LINE__, (unsigned long)dev);
#endif
	return ret;
}
#if 0//#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0))
#else
/* Set queue depth to get a reasonable value for q->nr_requests */
#define NAND_QUEUE_DEPTH 6

struct nand_queue_req {
	unsigned int			bytes_xfered;
	void *reserve;
};

static inline struct nand_queue_req *req_to_nand_queue_req(struct request *rq)
{
	return blk_mq_rq_to_pdu(rq);
}

static blk_status_t nand_mq_queue_rq(struct blk_mq_hw_ctx *hctx,
				    const struct blk_mq_queue_data *bd)
{
	//printk("<%s:%d> hctx: %lx, bd: %lx\n", __func__, __LINE__, hctx, bd);
	struct request *req = bd->rq;
	struct request_queue *q = req->q;
	int ret = BLK_STS_IOERR;
	struct nand_blk_ops *tr = q->queuedata;
	struct nand_blk_dev *dev = req->rq_disk->private_data;
	struct nand_queue_req *mqrq = req_to_nand_queue_req(req);
#ifdef NAND_WORER_NAME
	char worker_name[WORKER_DESC_LEN] = {0};
	struct worker *worker = current_wq_worker();
#endif
	unsigned int rq_data_len = 	blk_rq_bytes(req);

#ifdef NAND_WORER_NAME
	memcpy(worker_name, worker->desc, WORKER_DESC_LEN);
	set_worker_desc("nandw\n");
#endif

	if (!((req_op(req) == REQ_OP_FLUSH) || (req_op(req) == REQ_OP_DISCARD) || \
		(req_op(req) == REQ_OP_READ) || (req_op(req) == REQ_OP_WRITE))) {
		nand_dbg_err("unknow request %s,%d\n", __FUNCTION__, __LINE__);
		return BLK_STS_IOERR;
	}


	if (!mutex_trylock(&dev->lock)) {
		nand_dbg_inf("device busy\n");
		return BLK_STS_RESOURCE;
	}

	blk_mq_start_request(req);

	do {
		ret = do_blktrans_request(tr, dev, req);
		if (ret)
			break;
		cond_resched();
	} while (blk_update_request(req, BLK_STS_OK, blk_rq_cur_bytes(req)));


	if (ret) {
		ret = BLK_STS_IOERR;
		nand_dbg_err("%s,%d io error in do_blktrans_request\n", __FUNCTION__, __LINE__);
	} else {
		if ((req_op(req) == REQ_OP_FLUSH) || (req_op(req) == REQ_OP_DISCARD)) {
			blk_mq_end_request(req, BLK_STS_OK);
			ret = BLK_STS_OK;
			/*nand_dbg_inf("%s,%d\n", __FUNCTION__, __LINE__);*/
		} else if ((req_op(req) == REQ_OP_READ) || (req_op(req) == REQ_OP_WRITE)) {
			mqrq->bytes_xfered = rq_data_len;
			blk_mq_complete_request(req);
			ret = BLK_STS_OK;
			/*nand_dbg_inf("%s,%d %d\n", __FUNCTION__, __LINE__ , mqrq->bytes_xfered);*/
		} else {
			nand_dbg_err("unknow request %s,%d\n", __FUNCTION__, __LINE__);
			ret = BLK_STS_IOERR;
		}
	}
	mutex_unlock(&dev->lock);
#ifdef NAND_WORER_NAME
	memcpy(worker->desc, worker_name,  WORKER_DESC_LEN);
#endif
	return ret;
}

static void nand_blk_mq_complete_rq(struct request *req)
{
	struct nand_queue_req *mqrq = req_to_nand_queue_req(req);
	unsigned int nr_bytes = mqrq->bytes_xfered;

	if (nr_bytes) {
		__blk_mq_end_request(req, BLK_STS_OK);
		//	nand_dbg_err("%s,%d, has more data to comple %d\n", __FUNCTION__, __LINE__, blk_rq_bytes(req));
	} else if (!blk_rq_bytes(req)) {
		nand_dbg_err("%s,%d, io error request has not data %d req op %x\n", __FUNCTION__, __LINE__, blk_rq_bytes(req), req_op(req));
		__blk_mq_end_request(req, BLK_STS_IOERR);
	} else {
		nand_dbg_err("%s,%d, io error byte %d\n", __FUNCTION__, __LINE__, blk_rq_bytes(req));
		blk_mq_end_request(req, BLK_STS_IOERR);
	}
}

void nand_blk_mq_complete(struct request *req)
{
	//printk("<%s:%d> req: %lx\n", __func__, __LINE__, req);
	nand_blk_mq_complete_rq(req);
}

static enum blk_eh_timer_return nand_mq_timed_out(struct request *req,
						 bool reserved)
{
	/*struct request_queue *q = req->q;*/
	/*struct mmc_queue *mq = q->queuedata;*/
	/*unsigned long flags;*/
	int ret;

	nand_dbg_err("%s,%d, io timeout\n", __FUNCTION__, __LINE__);

	ret = BLK_EH_RESET_TIMER;
	return ret;
}

static const struct blk_mq_ops nand_mq_ops = {
	.queue_rq	= nand_mq_queue_rq,
	.init_request	= NULL,
	.exit_request	= NULL,
	.complete	= nand_blk_mq_complete,
	.timeout	= nand_mq_timed_out,
};

static void nand_setup_queue(struct nand_blk_ops *tr)
{
	unsigned hw_sectors = 128;
	blk_queue_write_cache(tr->rq, true, false);
	blk_queue_flag_set(QUEUE_FLAG_DISCARD, tr->rq);
	blk_queue_flag_set(QUEUE_FLAG_NONROT, tr->rq);
	blk_queue_flag_clear(QUEUE_FLAG_ADD_RANDOM, tr->rq);
	blk_queue_max_discard_sectors(tr->rq, UINT_MAX);

	/*blk_queue_bounce_limit(tr->rq, BLK_BOUNCE_HIGH);*/
	blk_queue_max_hw_sectors(tr->rq, hw_sectors);
	//blk_queue_max_segments(tr->rq, 1);
	blk_queue_logical_block_size(tr->rq, tr->blksize);
	blk_queue_max_segment_size(tr->rq,
			round_down(hw_sectors * 512, tr->blksize));
	dma_set_max_seg_size(ndfc_dev, queue_max_segment_size(tr->rq));
}

int nand_init_queue(struct nand_blk_ops *tr)
{
	int ret;

	memset(&tr->tag_set, 0, sizeof(tr->tag_set));
	tr->tag_set.ops = &nand_mq_ops;
	tr->tag_set.queue_depth = NAND_QUEUE_DEPTH;
	tr->tag_set.numa_node = NUMA_NO_NODE;
	tr->tag_set.flags = BLK_MQ_F_SHOULD_MERGE | BLK_MQ_F_BLOCKING;
	tr->tag_set.nr_hw_queues = 1;
	tr->tag_set.cmd_size = sizeof(struct nand_queue_req);
	tr->tag_set.driver_data = 0;

	ret = blk_mq_alloc_tag_set(&tr->tag_set);
	if (ret)
		return ret;

	tr->rq = blk_mq_init_queue(&tr->tag_set);
	if (!tr->rq) {
		unregister_blkdev(tr->major, tr->name);
		up(&nand_mutex);
		return -1;
	}

	blk_queue_rq_timeout(tr->rq, 60 * HZ);

	nand_setup_queue(tr);

	return 0;

free_tag_set:
	blk_mq_free_tag_set(&tr->tag_set);
	return ret;
}

#endif
/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
int nand_blk_register(struct nand_blk_ops *tr)
{
#if 0//#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0))
	int ret;

	struct _nand_phy_partition *phy_partition;

	down(&nand_mutex);

	ret = register_blkdev(tr->major, tr->name);
	if (ret) {
		nand_dbg_err("\nfaild to register blk device\n");
		up(&nand_mutex);
		return -1;
	}

	spin_lock_init(&tr->queue_lock);
	init_completion(&tr->thread_exit);
	init_waitqueue_head(&tr->thread_wq);
	sema_init(&tr->nand_ops_mutex, 1);

	tr->rq = blk_init_queue(mtd_blktrans_request, &tr->queue_lock);
	if (!tr->rq) {
		unregister_blkdev(tr->major, tr->name);
		up(&nand_mutex);
		return -1;
	}
#if 0
		ret = elevator_change(tr->rq, "noop");
		if (ret) {
			blk_cleanup_queue(tr->rq);
			return ret;
		}
#endif
	/* enable flush write cache req */
	blk_queue_write_cache(tr->rq, true, false);

	tr->rq->queuedata = tr;
	blk_queue_logical_block_size(tr->rq, tr->blksize);
	blk_queue_max_hw_sectors(tr->rq, 128);

	tr->thread = kthread_run(mtd_blktrans_thread, tr, "%s", tr->name);
	if (IS_ERR(tr->thread)) {
		ret = PTR_ERR(tr->thread);
		blk_cleanup_queue(tr->rq);
		unregister_blkdev(tr->major, tr->name);
		up(&nand_mutex);
		return ret;
	}

	queue_flag_set_unlocked(QUEUE_FLAG_DISCARD, tr->rq);
	tr->rq->limits.max_discard_sectors = UINT_MAX;

	INIT_LIST_HEAD(&tr->devs);
	tr->nftl_blk_head.nftl_blk_next = NULL;
	tr->nand_dev_head.nand_dev_next = NULL;

	phy_partition = get_head_phy_partition_from_nand_info(p_nand_info);

	while (phy_partition != NULL) {
		tr->add_dev(tr, phy_partition);
		phy_partition = get_next_phy_partition(phy_partition);
	}
	tr->rq->bypass_depth--;
	up(&nand_mutex);
#else
//TODO
	int ret;

	struct _nand_phy_partition *phy_partition;

	down(&nand_mutex);

	ret = register_blkdev(tr->major, tr->name);
	if (ret) {
		nand_dbg_err("\nfaild to register blk device\n");
		up(&nand_mutex);
		return -1;
	}

	init_completion(&tr->thread_exit);
	init_waitqueue_head(&tr->thread_wq);
	sema_init(&tr->nand_ops_mutex, 1);

	ret = nand_init_queue(tr);

#if 1
	tr->rq->queuedata = tr;
#else
	/* enable flush write cache req */
	blk_queue_write_cache(tr->rq, true, false);

	tr->rq->queuedata = tr;
	blk_queue_logical_block_size(tr->rq, tr->blksize);
	blk_queue_max_hw_sectors(tr->rq, 128);

	tr->thread = kthread_run(mtd_blktrans_thread, tr, "%s", tr->name);
	if (IS_ERR(tr->thread)) {
		ret = PTR_ERR(tr->thread);
		blk_cleanup_queue(tr->rq);
		unregister_blkdev(tr->major, tr->name);
		up(&nand_mutex);
		return ret;
	}
#endif

	INIT_LIST_HEAD(&tr->devs);
	tr->nftl_blk_head.nftl_blk_next = NULL;
	tr->nand_dev_head.nand_dev_next = NULL;

	phy_partition = get_head_phy_partition_from_nand_info(p_nand_info);

	printk("<%s:%d> phy_partition: %lx\n", __func__, __LINE__, (unsigned long)phy_partition);

	while (phy_partition != NULL) {
		tr->add_dev(tr, phy_partition);
		phy_partition = get_next_phy_partition(phy_partition);
		printk("<%s:%d> phy_partition: %lx\n", __func__, __LINE__, (unsigned long)phy_partition);
	}

	up(&nand_mutex);
	printk("<%s:%d> finish\n", __func__, __LINE__);
#endif
	return 0;
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
void nand_blk_unregister(struct nand_blk_ops *tr)
{

	down(&nand_mutex);
	/* Clean up the kernel thread */
	tr->quit = 1;
	wake_up(&tr->thread_wq);
	wait_for_completion(&tr->thread_exit);

	/* Remove it from the list of active majors */
	tr->remove_dev(tr);

	unregister_blkdev(tr->major, tr->name);
	blk_cleanup_queue(tr->rq);
	up(&nand_mutex);

	if (!list_empty(&tr->devs))
		BUG();
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
#if 0
int cal_partoff_within_disk(char *name, struct inode *i)
{
	struct gendisk *gd = i->i_bdev->bd_disk;
	int current_minor = MINOR(i->i_bdev->bd_dev);
	int index = current_minor & ((1 << mytr.minorbits) - 1);

	if (!index)
		return 0;

	return gd->part_tbl->part[index - 1]->start_sect;
}
#endif
/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
int init_blklayer(void)
{
	return nand_blk_register(&mytr);
}

int init_blklayer_for_dragonboard(void)
{
	int ret;
	struct nand_blk_ops *tr;

	tr = &mytr;

	dragonboard_test_flag = 1;

	down(&nand_mutex);

	ret = register_blkdev(tr->major, tr->name);
	if (ret) {
		nand_dbg_err("\nfaild to register blk device\n");
		up(&nand_mutex);
		return -1;
	}

	init_completion(&tr->thread_exit);
	init_waitqueue_head(&tr->thread_wq);
	sema_init(&tr->nand_ops_mutex, 1);

	INIT_LIST_HEAD(&tr->devs);
	tr->nftl_blk_head.nftl_blk_next = NULL;
	tr->nand_dev_head.nand_dev_next = NULL;

	tr->add_dev_test(tr);

	up(&nand_mutex);

	return 0;

}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
void exit_blklayer(void)
{
	nand_blk_unregister(&mytr);
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
int __init nand_drv_init(void)
{
	return init_blklayer();
}

/*****************************************************************************
*Name         :
*Description  :
*Parameter    :
*Return       :
*Note         :
*****************************************************************************/
void __exit nand_drv_exit(void)
{
	exit_blklayer();

}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("nand flash groups");
MODULE_DESCRIPTION("Generic NAND flash driver code");
