# SPDX-License-Identifier: GPL-2.0-or-later
#
# Allwinner audio codecs driver Makefile
#
# Copyright(c) 2022-2027 Allwinnertech Co., Ltd.

# Sunxi
snd-soc-ac107-objs := ac107.o
snd-soc-ac108-objs := ac108.o
# Fsm
snd-soc-fsm-objs   := fsm/fsm_core.o fsm/fsm_i2c.o fsm/fs16xx/fs1603.o

# Sunxi
obj-$(CONFIG_SND_SOC_AC107)	+= snd-soc-ac107.o
obj-$(CONFIG_SND_SOC_AC108)	+= snd-soc-ac108.o
# Fsm
obj-$(CONFIG_SND_SOC_FS16XX)+= snd-soc-fsm.o
