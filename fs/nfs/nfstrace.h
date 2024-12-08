/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2013 Trond Myklebust <Trond.Myklebust@netapp.com>
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM nfs

#if !defined(_TRACE_NFS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_NFS_H

#include <linux/tracepoint.h>
#include <linux/iversion.h>

#include <trace/misc/fs.h>
#include <trace/misc/nfs.h>
#include <trace/misc/sunrpc.h>

#define nfs_show_cache_validity(v) \
	__print_flags(v, "|", \
			{ NFS_INO_INVALID_DATA, "INVALID_DATA" }, \
			{ NFS_INO_INVALID_ATIME, "INVALID_ATIME" }, \
			{ NFS_INO_INVALID_ACCESS, "INVALID_ACCESS" }, \
			{ NFS_INO_INVALID_ACL, "INVALID_ACL" }, \
			{ NFS_INO_REVAL_PAGECACHE, "REVAL_PAGECACHE" }, \
			{ NFS_INO_REVAL_FORCED, "REVAL_FORCED" }, \
			{ NFS_INO_INVALID_LABEL, "INVALID_LABEL" }, \
			{ NFS_INO_INVALID_CHANGE, "INVALID_CHANGE" }, \
			{ NFS_INO_INVALID_CTIME, "INVALID_CTIME" }, \
			{ NFS_INO_INVALID_MTIME, "INVALID_MTIME" }, \
			{ NFS_INO_INVALID_SIZE, "INVALID_SIZE" }, \
			{ NFS_INO_INVALID_OTHER, "INVALID_OTHER" }, \
			{ NFS_INO_DATA_INVAL_DEFER, "DATA_INVAL_DEFER" }, \
			{ NFS_INO_INVALID_BLOCKS, "INVALID_BLOCKS" }, \
			{ NFS_INO_INVALID_XATTR, "INVALID_XATTR" }, \
			{ NFS_INO_INVALID_NLINK, "INVALID_NLINK" }, \
			{ NFS_INO_INVALID_MODE, "INVALID_MODE" })

#define nfs_show_nfsi_flags(v) \
	__print_flags(v, "|", \
			{ BIT(NFS_INO_ADVISE_RDPLUS), "ADVISE_RDPLUS" }, \
			{ BIT(NFS_INO_STALE), "STALE" }, \
			{ BIT(NFS_INO_ACL_LRU_SET), "ACL_LRU_SET" }, \
			{ BIT(NFS_INO_INVALIDATING), "INVALIDATING" }, \
			{ BIT(NFS_INO_FSCACHE), "FSCACHE" }, \
			{ BIT(NFS_INO_FSCACHE_LOCK), "FSCACHE_LOCK" }, \
			{ BIT(NFS_INO_LAYOUTCOMMIT), "NEED_LAYOUTCOMMIT" }, \
			{ BIT(NFS_INO_LAYOUTCOMMITTING), "LAYOUTCOMMIT" }, \
			{ BIT(NFS_INO_LAYOUTSTATS), "LAYOUTSTATS" }, \
			{ BIT(NFS_INO_ODIRECT), "ODIRECT" })

DECLARE_EVENT_CLASS(nfs_inode_event,
		TP_PROTO(
			const struct inode *inode
		),

		TP_ARGS(inode),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(u32, fhandle)
			__field(u64, fileid)
			__field(u64, version)
		),

		TP_fast_assign(
			const struct nfs_inode *nfsi = NFS_I(inode);
			__entry->dev = inode->i_sb->s_dev;
			__entry->fileid = nfsi->fileid;
			__entry->fhandle = nfs_fhandle_hash(&nfsi->fh);
			__entry->version = inode_peek_iversion_raw(inode);
		),

		TP_printk(
			"fileid=%02x:%02x:%llu fhandle=0x%08x version=%llu ",
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->fileid,
			__entry->fhandle,
			(unsigned long long)__entry->version
		)
);

DECLARE_EVENT_CLASS(nfs_inode_event_done,
		TP_PROTO(
			const struct inode *inode,
			int error
		),

		TP_ARGS(inode, error),

		TP_STRUCT__entry(
			__field(unsigned long, error)
			__field(dev_t, dev)
			__field(u32, fhandle)
			__field(unsigned char, type)
			__field(u64, fileid)
			__field(u64, version)
			__field(loff_t, size)
			__field(unsigned long, nfsi_flags)
			__field(unsigned long, cache_validity)
		),

		TP_fast_assign(
			const struct nfs_inode *nfsi = NFS_I(inode);
			__entry->error = error < 0 ? -error : 0;
			__entry->dev = inode->i_sb->s_dev;
			__entry->fileid = nfsi->fileid;
			__entry->fhandle = nfs_fhandle_hash(&nfsi->fh);
			__entry->type = nfs_umode_to_dtype(inode->i_mode);
			__entry->version = inode_peek_iversion_raw(inode);
			__entry->size = i_size_read(inode);
			__entry->nfsi_flags = nfsi->flags;
			__entry->cache_validity = nfsi->cache_validity;
		),

		TP_printk(
			"error=%ld (%s) fileid=%02x:%02x:%llu fhandle=0x%08x "
			"type=%u (%s) version=%llu size=%lld "
			"cache_validity=0x%lx (%s) nfs_flags=0x%lx (%s)",
			-__entry->error, show_nfs_status(__entry->error),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->fileid,
			__entry->fhandle,
			__entry->type,
			show_fs_dirent_type(__entry->type),
			(unsigned long long)__entry->version,
			(long long)__entry->size,
			__entry->cache_validity,
			nfs_show_cache_validity(__entry->cache_validity),
			__entry->nfsi_flags,
			nfs_show_nfsi_flags(__entry->nfsi_flags)
		)
);

#define DEFINE_NFS_INODE_EVENT(name) \
	DEFINE_EVENT(nfs_inode_event, name, \
			TP_PROTO( \
				const struct inode *inode \
			), \
			TP_ARGS(inode))
#define DEFINE_NFS_INODE_EVENT_DONE(name) \
	DEFINE_EVENT(nfs_inode_event_done, name, \
			TP_PROTO( \
				const struct inode *inode, \
				int error \
			), \
			TP_ARGS(inode, error))
DEFINE_NFS_INODE_EVENT(nfs_set_inode_stale);
DEFINE_NFS_INODE_EVENT(nfs_refresh_inode_enter);
DEFINE_NFS_INODE_EVENT_DONE(nfs_refresh_inode_exit);
DEFINE_NFS_INODE_EVENT(nfs_revalidate_inode_enter);
DEFINE_NFS_INODE_EVENT_DONE(nfs_revalidate_inode_exit);
DEFINE_NFS_INODE_EVENT(nfs_invalidate_mapping_enter);
DEFINE_NFS_INODE_EVENT_DONE(nfs_invalidate_mapping_exit);
DEFINE_NFS_INODE_EVENT(nfs_getattr_enter);
DEFINE_NFS_INODE_EVENT_DONE(nfs_getattr_exit);
DEFINE_NFS_INODE_EVENT(nfs_setattr_enter);
DEFINE_NFS_INODE_EVENT_DONE(nfs_setattr_exit);
DEFINE_NFS_INODE_EVENT(nfs_writeback_page_enter);
DEFINE_NFS_INODE_EVENT_DONE(nfs_writeback_page_exit);
DEFINE_NFS_INODE_EVENT(nfs_writeback_inode_enter);
DEFINE_NFS_INODE_EVENT_DONE(nfs_writeback_inode_exit);
DEFINE_NFS_INODE_EVENT(nfs_fsync_enter);
DEFINE_NFS_INODE_EVENT_DONE(nfs_fsync_exit);
DEFINE_NFS_INODE_EVENT(nfs_access_enter);
DEFINE_NFS_INODE_EVENT_DONE(nfs_set_cache_invalid);

TRACE_EVENT(nfs_access_exit,
		TP_PROTO(
			const struct inode *inode,
			unsigned int mask,
			unsigned int permitted,
			int error
		),

		TP_ARGS(inode, mask, permitted, error),

		TP_STRUCT__entry(
			__field(unsigned long, error)
			__field(dev_t, dev)
			__field(u32, fhandle)
			__field(unsigned char, type)
			__field(u64, fileid)
			__field(u64, version)
			__field(loff_t, size)
			__field(unsigned long, nfsi_flags)
			__field(unsigned long, cache_validity)
			__field(unsigned int, mask)
			__field(unsigned int, permitted)
		),

		TP_fast_assign(
			const struct nfs_inode *nfsi = NFS_I(inode);
			__entry->error = error < 0 ? -error : 0;
			__entry->dev = inode->i_sb->s_dev;
			__entry->fileid = nfsi->fileid;
			__entry->fhandle = nfs_fhandle_hash(&nfsi->fh);
			__entry->type = nfs_umode_to_dtype(inode->i_mode);
			__entry->version = inode_peek_iversion_raw(inode);
			__entry->size = i_size_read(inode);
			__entry->nfsi_flags = nfsi->flags;
			__entry->cache_validity = nfsi->cache_validity;
			__entry->mask = mask;
			__entry->permitted = permitted;
		),

		TP_printk(
			"error=%ld (%s) fileid=%02x:%02x:%llu fhandle=0x%08x "
			"type=%u (%s) version=%llu size=%lld "
			"cache_validity=0x%lx (%s) nfs_flags=0x%lx (%s) "
			"mask=0x%x permitted=0x%x",
			-__entry->error, show_nfs_status(__entry->error),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->fileid,
			__entry->fhandle,
			__entry->type,
			show_fs_dirent_type(__entry->type),
			(unsigned long long)__entry->version,
			(long long)__entry->size,
			__entry->cache_validity,
			nfs_show_cache_validity(__entry->cache_validity),
			__entry->nfsi_flags,
			nfs_show_nfsi_flags(__entry->nfsi_flags),
			__entry->mask, __entry->permitted
		)
);

DECLARE_EVENT_CLASS(nfs_lookup_event,
		TP_PROTO(
			const struct inode *dir,
			const struct dentry *dentry,
			unsigned int flags
		),

		TP_ARGS(dir, dentry, flags),

		TP_STRUCT__entry(
			__field(unsigned long, flags)
			__field(dev_t, dev)
			__field(u64, dir)
			__string(name, dentry->d_name.name)
		),

		TP_fast_assign(
			__entry->dev = dir->i_sb->s_dev;
			__entry->dir = NFS_FILEID(dir);
			__entry->flags = flags;
			__assign_str(name, dentry->d_name.name);
		),

		TP_printk(
			"flags=0x%lx (%s) name=%02x:%02x:%llu/%s",
			__entry->flags,
			show_fs_lookup_flags(__entry->flags),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->dir,
			__get_str(name)
		)
);

#define DEFINE_NFS_LOOKUP_EVENT(name) \
	DEFINE_EVENT(nfs_lookup_event, name, \
			TP_PROTO( \
				const struct inode *dir, \
				const struct dentry *dentry, \
				unsigned int flags \
			), \
			TP_ARGS(dir, dentry, flags))

DECLARE_EVENT_CLASS(nfs_lookup_event_done,
		TP_PROTO(
			const struct inode *dir,
			const struct dentry *dentry,
			unsigned int flags,
			int error
		),

		TP_ARGS(dir, dentry, flags, error),

		TP_STRUCT__entry(
			__field(unsigned long, error)
			__field(unsigned long, flags)
			__field(dev_t, dev)
			__field(u64, dir)
			__string(name, dentry->d_name.name)
		),

		TP_fast_assign(
			__entry->dev = dir->i_sb->s_dev;
			__entry->dir = NFS_FILEID(dir);
			__entry->error = error < 0 ? -error : 0;
			__entry->flags = flags;
			__assign_str(name, dentry->d_name.name);
		),

		TP_printk(
			"error=%ld (%s) flags=0x%lx (%s) name=%02x:%02x:%llu/%s",
			-__entry->error, show_nfs_status(__entry->error),
			__entry->flags,
			show_fs_lookup_flags(__entry->flags),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->dir,
			__get_str(name)
		)
);

#define DEFINE_NFS_LOOKUP_EVENT_DONE(name) \
	DEFINE_EVENT(nfs_lookup_event_done, name, \
			TP_PROTO( \
				const struct inode *dir, \
				const struct dentry *dentry, \
				unsigned int flags, \
				int error \
			), \
			TP_ARGS(dir, dentry, flags, error))

DEFINE_NFS_LOOKUP_EVENT(nfs_lookup_enter);
DEFINE_NFS_LOOKUP_EVENT_DONE(nfs_lookup_exit);
DEFINE_NFS_LOOKUP_EVENT(nfs_lookup_revalidate_enter);
DEFINE_NFS_LOOKUP_EVENT_DONE(nfs_lookup_revalidate_exit);

TRACE_EVENT(nfs_atomic_open_enter,
		TP_PROTO(
			const struct inode *dir,
			const struct nfs_open_context *ctx,
			unsigned int flags
		),

		TP_ARGS(dir, ctx, flags),

		TP_STRUCT__entry(
			__field(unsigned long, flags)
			__field(unsigned long, fmode)
			__field(dev_t, dev)
			__field(u64, dir)
			__string(name, ctx->dentry->d_name.name)
		),

		TP_fast_assign(
			__entry->dev = dir->i_sb->s_dev;
			__entry->dir = NFS_FILEID(dir);
			__entry->flags = flags;
			__entry->fmode = (__force unsigned long)ctx->mode;
			__assign_str(name, ctx->dentry->d_name.name);
		),

		TP_printk(
			"flags=0x%lx (%s) fmode=%s name=%02x:%02x:%llu/%s",
			__entry->flags,
			show_fs_fcntl_open_flags(__entry->flags),
			show_fs_fmode_flags(__entry->fmode),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->dir,
			__get_str(name)
		)
);

TRACE_EVENT(nfs_atomic_open_exit,
		TP_PROTO(
			const struct inode *dir,
			const struct nfs_open_context *ctx,
			unsigned int flags,
			int error
		),

		TP_ARGS(dir, ctx, flags, error),

		TP_STRUCT__entry(
			__field(unsigned long, error)
			__field(unsigned long, flags)
			__field(unsigned long, fmode)
			__field(dev_t, dev)
			__field(u64, dir)
			__string(name, ctx->dentry->d_name.name)
		),

		TP_fast_assign(
			__entry->error = -error;
			__entry->dev = dir->i_sb->s_dev;
			__entry->dir = NFS_FILEID(dir);
			__entry->flags = flags;
			__entry->fmode = (__force unsigned long)ctx->mode;
			__assign_str(name, ctx->dentry->d_name.name);
		),

		TP_printk(
			"error=%ld (%s) flags=0x%lx (%s) fmode=%s "
			"name=%02x:%02x:%llu/%s",
			-__entry->error, show_nfs_status(__entry->error),
			__entry->flags,
			show_fs_fcntl_open_flags(__entry->flags),
			show_fs_fmode_flags(__entry->fmode),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->dir,
			__get_str(name)
		)
);

TRACE_EVENT(nfs_create_enter,
		TP_PROTO(
			const struct inode *dir,
			const struct dentry *dentry,
			unsigned int flags
		),

		TP_ARGS(dir, dentry, flags),

		TP_STRUCT__entry(
			__field(unsigned long, flags)
			__field(dev_t, dev)
			__field(u64, dir)
			__string(name, dentry->d_name.name)
		),

		TP_fast_assign(
			__entry->dev = dir->i_sb->s_dev;
			__entry->dir = NFS_FILEID(dir);
			__entry->flags = flags;
			__assign_str(name, dentry->d_name.name);
		),

		TP_printk(
			"flags=0x%lx (%s) name=%02x:%02x:%llu/%s",
			__entry->flags,
			show_fs_fcntl_open_flags(__entry->flags),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->dir,
			__get_str(name)
		)
);

TRACE_EVENT(nfs_create_exit,
		TP_PROTO(
			const struct inode *dir,
			const struct dentry *dentry,
			unsigned int flags,
			int error
		),

		TP_ARGS(dir, dentry, flags, error),

		TP_STRUCT__entry(
			__field(unsigned long, error)
			__field(unsigned long, flags)
			__field(dev_t, dev)
			__field(u64, dir)
			__string(name, dentry->d_name.name)
		),

		TP_fast_assign(
			__entry->error = -error;
			__entry->dev = dir->i_sb->s_dev;
			__entry->dir = NFS_FILEID(dir);
			__entry->flags = flags;
			__assign_str(name, dentry->d_name.name);
		),

		TP_printk(
			"error=%ld (%s) flags=0x%lx (%s) name=%02x:%02x:%llu/%s",
			-__entry->error, show_nfs_status(__entry->error),
			__entry->flags,
			show_fs_fcntl_open_flags(__entry->flags),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->dir,
			__get_str(name)
		)
);

DECLARE_EVENT_CLASS(nfs_directory_event,
		TP_PROTO(
			const struct inode *dir,
			const struct dentry *dentry
		),

		TP_ARGS(dir, dentry),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(u64, dir)
			__string(name, dentry->d_name.name)
		),

		TP_fast_assign(
			__entry->dev = dir->i_sb->s_dev;
			__entry->dir = NFS_FILEID(dir);
			__assign_str(name, dentry->d_name.name);
		),

		TP_printk(
			"name=%02x:%02x:%llu/%s",
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->dir,
			__get_str(name)
		)
);

#define DEFINE_NFS_DIRECTORY_EVENT(name) \
	DEFINE_EVENT(nfs_directory_event, name, \
			TP_PROTO( \
				const struct inode *dir, \
				const struct dentry *dentry \
			), \
			TP_ARGS(dir, dentry))

DECLARE_EVENT_CLASS(nfs_directory_event_done,
		TP_PROTO(
			const struct inode *dir,
			const struct dentry *dentry,
			int error
		),

		TP_ARGS(dir, dentry, error),

		TP_STRUCT__entry(
			__field(unsigned long, error)
			__field(dev_t, dev)
			__field(u64, dir)
			__string(name, dentry->d_name.name)
		),

		TP_fast_assign(
			__entry->dev = dir->i_sb->s_dev;
			__entry->dir = NFS_FILEID(dir);
			__entry->error = error < 0 ? -error : 0;
			__assign_str(name, dentry->d_name.name);
		),

		TP_printk(
			"error=%ld (%s) name=%02x:%02x:%llu/%s",
			-__entry->error, show_nfs_status(__entry->error),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->dir,
			__get_str(name)
		)
);

#define DEFINE_NFS_DIRECTORY_EVENT_DONE(name) \
	DEFINE_EVENT(nfs_directory_event_done, name, \
			TP_PROTO( \
				const struct inode *dir, \
				const struct dentry *dentry, \
				int error \
			), \
			TP_ARGS(dir, dentry, error))

DEFINE_NFS_DIRECTORY_EVENT(nfs_mknod_enter);
DEFINE_NFS_DIRECTORY_EVENT_DONE(nfs_mknod_exit);
DEFINE_NFS_DIRECTORY_EVENT(nfs_mkdir_enter);
DEFINE_NFS_DIRECTORY_EVENT_DONE(nfs_mkdir_exit);
DEFINE_NFS_DIRECTORY_EVENT(nfs_rmdir_enter);
DEFINE_NFS_DIRECTORY_EVENT_DONE(nfs_rmdir_exit);
DEFINE_NFS_DIRECTORY_EVENT(nfs_remove_enter);
DEFINE_NFS_DIRECTORY_EVENT_DONE(nfs_remove_exit);
DEFINE_NFS_DIRECTORY_EVENT(nfs_unlink_enter);
DEFINE_NFS_DIRECTORY_EVENT_DONE(nfs_unlink_exit);
DEFINE_NFS_DIRECTORY_EVENT(nfs_symlink_enter);
DEFINE_NFS_DIRECTORY_EVENT_DONE(nfs_symlink_exit);

TRACE_EVENT(nfs_link_enter,
		TP_PROTO(
			const struct inode *inode,
			const struct inode *dir,
			const struct dentry *dentry
		),

		TP_ARGS(inode, dir, dentry),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(u64, fileid)
			__field(u64, dir)
			__string(name, dentry->d_name.name)
		),

		TP_fast_assign(
			__entry->dev = inode->i_sb->s_dev;
			__entry->fileid = NFS_FILEID(inode);
			__entry->dir = NFS_FILEID(dir);
			__assign_str(name, dentry->d_name.name);
		),

		TP_printk(
			"fileid=%02x:%02x:%llu name=%02x:%02x:%llu/%s",
			MAJOR(__entry->dev), MINOR(__entry->dev),
			__entry->fileid,
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->dir,
			__get_str(name)
		)
);

TRACE_EVENT(nfs_link_exit,
		TP_PROTO(
			const struct inode *inode,
			const struct inode *dir,
			const struct dentry *dentry,
			int error
		),

		TP_ARGS(inode, dir, dentry, error),

		TP_STRUCT__entry(
			__field(unsigned long, error)
			__field(dev_t, dev)
			__field(u64, fileid)
			__field(u64, dir)
			__string(name, dentry->d_name.name)
		),

		TP_fast_assign(
			__entry->dev = inode->i_sb->s_dev;
			__entry->fileid = NFS_FILEID(inode);
			__entry->dir = NFS_FILEID(dir);
			__entry->error = error < 0 ? -error : 0;
			__assign_str(name, dentry->d_name.name);
		),

		TP_printk(
			"error=%ld (%s) fileid=%02x:%02x:%llu name=%02x:%02x:%llu/%s",
			-__entry->error, show_nfs_status(__entry->error),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			__entry->fileid,
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->dir,
			__get_str(name)
		)
);

DECLARE_EVENT_CLASS(nfs_rename_event,
		TP_PROTO(
			const struct inode *old_dir,
			const struct dentry *old_dentry,
			const struct inode *new_dir,
			const struct dentry *new_dentry
		),

		TP_ARGS(old_dir, old_dentry, new_dir, new_dentry),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(u64, old_dir)
			__field(u64, new_dir)
			__string(old_name, old_dentry->d_name.name)
			__string(new_name, new_dentry->d_name.name)
		),

		TP_fast_assign(
			__entry->dev = old_dir->i_sb->s_dev;
			__entry->old_dir = NFS_FILEID(old_dir);
			__entry->new_dir = NFS_FILEID(new_dir);
			__assign_str(old_name, old_dentry->d_name.name);
			__assign_str(new_name, new_dentry->d_name.name);
		),

		TP_printk(
			"old_name=%02x:%02x:%llu/%s new_name=%02x:%02x:%llu/%s",
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->old_dir,
			__get_str(old_name),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->new_dir,
			__get_str(new_name)
		)
);
#define DEFINE_NFS_RENAME_EVENT(name) \
	DEFINE_EVENT(nfs_rename_event, name, \
			TP_PROTO( \
				const struct inode *old_dir, \
				const struct dentry *old_dentry, \
				const struct inode *new_dir, \
				const struct dentry *new_dentry \
			), \
			TP_ARGS(old_dir, old_dentry, new_dir, new_dentry))

DECLARE_EVENT_CLASS(nfs_rename_event_done,
		TP_PROTO(
			const struct inode *old_dir,
			const struct dentry *old_dentry,
			const struct inode *new_dir,
			const struct dentry *new_dentry,
			int error
		),

		TP_ARGS(old_dir, old_dentry, new_dir, new_dentry, error),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(unsigned long, error)
			__field(u64, old_dir)
			__string(old_name, old_dentry->d_name.name)
			__field(u64, new_dir)
			__string(new_name, new_dentry->d_name.name)
		),

		TP_fast_assign(
			__entry->dev = old_dir->i_sb->s_dev;
			__entry->error = -error;
			__entry->old_dir = NFS_FILEID(old_dir);
			__entry->new_dir = NFS_FILEID(new_dir);
			__assign_str(old_name, old_dentry->d_name.name);
			__assign_str(new_name, new_dentry->d_name.name);
		),

		TP_printk(
			"error=%ld (%s) old_name=%02x:%02x:%llu/%s "
			"new_name=%02x:%02x:%llu/%s",
			-__entry->error, show_nfs_status(__entry->error),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->old_dir,
			__get_str(old_name),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->new_dir,
			__get_str(new_name)
		)
);
#define DEFINE_NFS_RENAME_EVENT_DONE(name) \
	DEFINE_EVENT(nfs_rename_event_done, name, \
			TP_PROTO( \
				const struct inode *old_dir, \
				const struct dentry *old_dentry, \
				const struct inode *new_dir, \
				const struct dentry *new_dentry, \
				int error \
			), \
			TP_ARGS(old_dir, old_dentry, new_dir, \
				new_dentry, error))

DEFINE_NFS_RENAME_EVENT(nfs_rename_enter);
DEFINE_NFS_RENAME_EVENT_DONE(nfs_rename_exit);

DEFINE_NFS_RENAME_EVENT_DONE(nfs_sillyrename_rename);

TRACE_EVENT(nfs_sillyrename_unlink,
		TP_PROTO(
			const struct nfs_unlinkdata *data,
			int error
		),

		TP_ARGS(data, error),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(unsigned long, error)
			__field(u64, dir)
			__dynamic_array(char, name, data->args.name.len + 1)
		),

		TP_fast_assign(
			struct inode *dir = d_inode(data->dentry->d_parent);
			size_t len = data->args.name.len;
			__entry->dev = dir->i_sb->s_dev;
			__entry->dir = NFS_FILEID(dir);
			__entry->error = -error;
			memcpy(__get_str(name),
				data->args.name.name, len);
			__get_str(name)[len] = 0;
		),

		TP_printk(
			"error=%ld (%s) name=%02x:%02x:%llu/%s",
			-__entry->error, show_nfs_status(__entry->error),
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->dir,
			__get_str(name)
		)
);

TRACE_EVENT(nfs_initiate_read,
		TP_PROTO(
			const struct nfs_pgio_header *hdr
		),

		TP_ARGS(hdr),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(u32, fhandle)
			__field(u64, fileid)
			__field(loff_t, offset)
			__field(u32, count)
		),

		TP_fast_assign(
			const struct inode *inode = hdr->inode;
			const struct nfs_inode *nfsi = NFS_I(inode);
			const struct nfs_fh *fh = hdr->args.fh ?
						  hdr->args.fh : &nfsi->fh;

			__entry->offset = hdr->args.offset;
			__entry->count = hdr->args.count;
			__entry->dev = inode->i_sb->s_dev;
			__entry->fileid = nfsi->fileid;
			__entry->fhandle = nfs_fhandle_hash(fh);
		),

		TP_printk(
			"fileid=%02x:%02x:%llu fhandle=0x%08x "
			"offset=%lld count=%u",
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->fileid,
			__entry->fhandle,
			(long long)__entry->offset, __entry->count
		)
);

TRACE_EVENT(nfs_readpage_done,
		TP_PROTO(
			const struct rpc_task *task,
			const struct nfs_pgio_header *hdr
		),

		TP_ARGS(task, hdr),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(u32, fhandle)
			__field(u64, fileid)
			__field(loff_t, offset)
			__field(u32, arg_count)
			__field(u32, res_count)
			__field(bool, eof)
			__field(int, status)
		),

		TP_fast_assign(
			const struct inode *inode = hdr->inode;
			const struct nfs_inode *nfsi = NFS_I(inode);
			const struct nfs_fh *fh = hdr->args.fh ?
						  hdr->args.fh : &nfsi->fh;

			__entry->status = task->tk_status;
			__entry->offset = hdr->args.offset;
			__entry->arg_count = hdr->args.count;
			__entry->res_count = hdr->res.count;
			__entry->eof = hdr->res.eof;
			__entry->dev = inode->i_sb->s_dev;
			__entry->fileid = nfsi->fileid;
			__entry->fhandle = nfs_fhandle_hash(fh);
		),

		TP_printk(
			"fileid=%02x:%02x:%llu fhandle=0x%08x "
			"offset=%lld count=%u res=%u status=%d%s",
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->fileid,
			__entry->fhandle,
			(long long)__entry->offset, __entry->arg_count,
			__entry->res_count, __entry->status,
			__entry->eof ? " eof" : ""
		)
);

TRACE_EVENT(nfs_readpage_short,
		TP_PROTO(
			const struct rpc_task *task,
			const struct nfs_pgio_header *hdr
		),

		TP_ARGS(task, hdr),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(u32, fhandle)
			__field(u64, fileid)
			__field(loff_t, offset)
			__field(u32, arg_count)
			__field(u32, res_count)
			__field(bool, eof)
			__field(int, status)
		),

		TP_fast_assign(
			const struct inode *inode = hdr->inode;
			const struct nfs_inode *nfsi = NFS_I(inode);
			const struct nfs_fh *fh = hdr->args.fh ?
						  hdr->args.fh : &nfsi->fh;

			__entry->status = task->tk_status;
			__entry->offset = hdr->args.offset;
			__entry->arg_count = hdr->args.count;
			__entry->res_count = hdr->res.count;
			__entry->eof = hdr->res.eof;
			__entry->dev = inode->i_sb->s_dev;
			__entry->fileid = nfsi->fileid;
			__entry->fhandle = nfs_fhandle_hash(fh);
		),

		TP_printk(
			"fileid=%02x:%02x:%llu fhandle=0x%08x "
			"offset=%lld count=%u res=%u status=%d%s",
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->fileid,
			__entry->fhandle,
			(long long)__entry->offset, __entry->arg_count,
			__entry->res_count, __entry->status,
			__entry->eof ? " eof" : ""
		)
);

TRACE_EVENT(nfs_pgio_error,
	TP_PROTO(
		const struct nfs_pgio_header *hdr,
		int error,
		loff_t pos
	),

	TP_ARGS(hdr, error, pos),

	TP_STRUCT__entry(
		__field(dev_t, dev)
		__field(u32, fhandle)
		__field(u64, fileid)
		__field(loff_t, offset)
		__field(u32, arg_count)
		__field(u32, res_count)
		__field(loff_t, pos)
		__field(int, status)
	),

	TP_fast_assign(
		const struct inode *inode = hdr->inode;
		const struct nfs_inode *nfsi = NFS_I(inode);
		const struct nfs_fh *fh = hdr->args.fh ?
					  hdr->args.fh : &nfsi->fh;

		__entry->status = error;
		__entry->offset = hdr->args.offset;
		__entry->arg_count = hdr->args.count;
		__entry->res_count = hdr->res.count;
		__entry->dev = inode->i_sb->s_dev;
		__entry->fileid = nfsi->fileid;
		__entry->fhandle = nfs_fhandle_hash(fh);
	),

	TP_printk("fileid=%02x:%02x:%llu fhandle=0x%08x "
		  "offset=%lld count=%u res=%u pos=%llu status=%d",
		MAJOR(__entry->dev), MINOR(__entry->dev),
		(unsigned long long)__entry->fileid, __entry->fhandle,
		(long long)__entry->offset, __entry->arg_count, __entry->res_count,
		__entry->pos, __entry->status
	)
);

TRACE_EVENT(nfs_initiate_write,
		TP_PROTO(
			const struct nfs_pgio_header *hdr
		),

		TP_ARGS(hdr),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(u32, fhandle)
			__field(u64, fileid)
			__field(loff_t, offset)
			__field(u32, count)
			__field(unsigned long, stable)
		),

		TP_fast_assign(
			const struct inode *inode = hdr->inode;
			const struct nfs_inode *nfsi = NFS_I(inode);
			const struct nfs_fh *fh = hdr->args.fh ?
						  hdr->args.fh : &nfsi->fh;

			__entry->offset = hdr->args.offset;
			__entry->count = hdr->args.count;
			__entry->stable = hdr->args.stable;
			__entry->dev = inode->i_sb->s_dev;
			__entry->fileid = nfsi->fileid;
			__entry->fhandle = nfs_fhandle_hash(fh);
		),

		TP_printk(
			"fileid=%02x:%02x:%llu fhandle=0x%08x "
			"offset=%lld count=%u stable=%s",
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->fileid,
			__entry->fhandle,
			(long long)__entry->offset, __entry->count,
			show_nfs_stable_how(__entry->stable)
		)
);

TRACE_EVENT(nfs_writeback_done,
		TP_PROTO(
			const struct rpc_task *task,
			const struct nfs_pgio_header *hdr
		),

		TP_ARGS(task, hdr),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(u32, fhandle)
			__field(u64, fileid)
			__field(loff_t, offset)
			__field(u32, arg_count)
			__field(u32, res_count)
			__field(int, status)
			__field(unsigned long, stable)
			__array(char, verifier, NFS4_VERIFIER_SIZE)
		),

		TP_fast_assign(
			const struct inode *inode = hdr->inode;
			const struct nfs_inode *nfsi = NFS_I(inode);
			const struct nfs_fh *fh = hdr->args.fh ?
						  hdr->args.fh : &nfsi->fh;
			const struct nfs_writeverf *verf = hdr->res.verf;

			__entry->status = task->tk_status;
			__entry->offset = hdr->args.offset;
			__entry->arg_count = hdr->args.count;
			__entry->res_count = hdr->res.count;
			__entry->stable = verf->committed;
			memcpy(__entry->verifier,
				&verf->verifier,
				NFS4_VERIFIER_SIZE);
			__entry->dev = inode->i_sb->s_dev;
			__entry->fileid = nfsi->fileid;
			__entry->fhandle = nfs_fhandle_hash(fh);
		),

		TP_printk(
			"fileid=%02x:%02x:%llu fhandle=0x%08x "
			"offset=%lld count=%u res=%u status=%d stable=%s "
			"verifier=%s",
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->fileid,
			__entry->fhandle,
			(long long)__entry->offset, __entry->arg_count,
			__entry->res_count, __entry->status,
			show_nfs_stable_how(__entry->stable),
			show_nfs4_verifier(__entry->verifier)
		)
);

DECLARE_EVENT_CLASS(nfs_page_error_class,
		TP_PROTO(
			const struct nfs_page *req,
			int error
		),

		TP_ARGS(req, error),

		TP_STRUCT__entry(
			__field(const void *, req)
			__field(pgoff_t, index)
			__field(unsigned int, offset)
			__field(unsigned int, pgbase)
			__field(unsigned int, bytes)
			__field(int, error)
		),

		TP_fast_assign(
			__entry->req = req;
			__entry->index = req->wb_index;
			__entry->offset = req->wb_offset;
			__entry->pgbase = req->wb_pgbase;
			__entry->bytes = req->wb_bytes;
			__entry->error = error;
		),

		TP_printk(
			"req=%p index=%lu offset=%u pgbase=%u bytes=%u error=%d",
			__entry->req, __entry->index, __entry->offset,
			__entry->pgbase, __entry->bytes, __entry->error
		)
);

#define DEFINE_NFS_PAGEERR_EVENT(name) \
	DEFINE_EVENT(nfs_page_error_class, name, \
			TP_PROTO( \
				const struct nfs_page *req, \
				int error \
			), \
			TP_ARGS(req, error))

DEFINE_NFS_PAGEERR_EVENT(nfs_write_error);
DEFINE_NFS_PAGEERR_EVENT(nfs_comp_error);
DEFINE_NFS_PAGEERR_EVENT(nfs_commit_error);

TRACE_EVENT(nfs_initiate_commit,
		TP_PROTO(
			const struct nfs_commit_data *data
		),

		TP_ARGS(data),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(u32, fhandle)
			__field(u64, fileid)
			__field(loff_t, offset)
			__field(u32, count)
		),

		TP_fast_assign(
			const struct inode *inode = data->inode;
			const struct nfs_inode *nfsi = NFS_I(inode);
			const struct nfs_fh *fh = data->args.fh ?
						  data->args.fh : &nfsi->fh;

			__entry->offset = data->args.offset;
			__entry->count = data->args.count;
			__entry->dev = inode->i_sb->s_dev;
			__entry->fileid = nfsi->fileid;
			__entry->fhandle = nfs_fhandle_hash(fh);
		),

		TP_printk(
			"fileid=%02x:%02x:%llu fhandle=0x%08x "
			"offset=%lld count=%u",
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->fileid,
			__entry->fhandle,
			(long long)__entry->offset, __entry->count
		)
);

TRACE_EVENT(nfs_commit_done,
		TP_PROTO(
			const struct rpc_task *task,
			const struct nfs_commit_data *data
		),

		TP_ARGS(task, data),

		TP_STRUCT__entry(
			__field(dev_t, dev)
			__field(u32, fhandle)
			__field(u64, fileid)
			__field(loff_t, offset)
			__field(int, status)
			__field(unsigned long, stable)
			__array(char, verifier, NFS4_VERIFIER_SIZE)
		),

		TP_fast_assign(
			const struct inode *inode = data->inode;
			const struct nfs_inode *nfsi = NFS_I(inode);
			const struct nfs_fh *fh = data->args.fh ?
						  data->args.fh : &nfsi->fh;
			const struct nfs_writeverf *verf = data->res.verf;

			__entry->status = task->tk_status;
			__entry->offset = data->args.offset;
			__entry->stable = verf->committed;
			memcpy(__entry->verifier,
				&verf->verifier,
				NFS4_VERIFIER_SIZE);
			__entry->dev = inode->i_sb->s_dev;
			__entry->fileid = nfsi->fileid;
			__entry->fhandle = nfs_fhandle_hash(fh);
		),

		TP_printk(
			"fileid=%02x:%02x:%llu fhandle=0x%08x "
			"offset=%lld status=%d stable=%s verifier=%s",
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->fileid,
			__entry->fhandle,
			(long long)__entry->offset, __entry->status,
			show_nfs_stable_how(__entry->stable),
			show_nfs4_verifier(__entry->verifier)
		)
);

TRACE_EVENT(nfs_fh_to_dentry,
		TP_PROTO(
			const struct super_block *sb,
			const struct nfs_fh *fh,
			u64 fileid,
			int error
		),

		TP_ARGS(sb, fh, fileid, error),

		TP_STRUCT__entry(
			__field(int, error)
			__field(dev_t, dev)
			__field(u32, fhandle)
			__field(u64, fileid)
		),

		TP_fast_assign(
			__entry->error = error;
			__entry->dev = sb->s_dev;
			__entry->fileid = fileid;
			__entry->fhandle = nfs_fhandle_hash(fh);
		),

		TP_printk(
			"error=%d fileid=%02x:%02x:%llu fhandle=0x%08x ",
			__entry->error,
			MAJOR(__entry->dev), MINOR(__entry->dev),
			(unsigned long long)__entry->fileid,
			__entry->fhandle
		)
);

DECLARE_EVENT_CLASS(nfs_xdr_event,
		TP_PROTO(
			const struct xdr_stream *xdr,
			int error
		),

		TP_ARGS(xdr, error),

		TP_STRUCT__entry(
			__field(unsigned int, task_id)
			__field(unsigned int, client_id)
			__field(u32, xid)
			__field(int, version)
			__field(unsigned long, error)
			__string(program,
				 xdr->rqst->rq_task->tk_client->cl_program->name)
			__string(procedure,
				 xdr->rqst->rq_task->tk_msg.rpc_proc->p_name)
		),

		TP_fast_assign(
			const struct rpc_rqst *rqstp = xdr->rqst;
			const struct rpc_task *task = rqstp->rq_task;

			__entry->task_id = task->tk_pid;
			__entry->client_id = task->tk_client->cl_clid;
			__entry->xid = be32_to_cpu(rqstp->rq_xid);
			__entry->version = task->tk_client->cl_vers;
			__entry->error = error;
			__assign_str(program,
				     task->tk_client->cl_program->name);
			__assign_str(procedure, task->tk_msg.rpc_proc->p_name);
		),

		TP_printk(SUNRPC_TRACE_TASK_SPECIFIER
			  " xid=0x%08x %sv%d %s error=%ld (%s)",
			__entry->task_id, __entry->client_id, __entry->xid,
			__get_str(program), __entry->version,
			__get_str(procedure), -__entry->error,
			show_nfs_status(__entry->error)
		)
);
#define DEFINE_NFS_XDR_EVENT(name) \
	DEFINE_EVENT(nfs_xdr_event, name, \
			TP_PROTO( \
				const struct xdr_stream *xdr, \
				int error \
			), \
			TP_ARGS(xdr, error))
DEFINE_NFS_XDR_EVENT(nfs_xdr_status);
DEFINE_NFS_XDR_EVENT(nfs_xdr_bad_filehandle);

#endif /* _TRACE_NFS_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE nfstrace
/* This part must be outside protection */
#include <trace/define_trace.h>
