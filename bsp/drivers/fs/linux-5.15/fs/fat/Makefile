# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux fat filesystem support.
#

obj-$(CONFIG_AW_FAT_FS) += fat.o
obj-$(CONFIG_AW_VFAT_FS) += vfat.o
obj-$(CONFIG_AW_MSDOS_FS) += msdos.o

fat-y := cache.o dir.o fatent.o file.o inode.o misc.o nfs.o
vfat-y := namei_vfat.o
msdos-y := namei_msdos.o

obj-$(CONFIG_FAT_KUNIT_TEST) += fat_test.o
