ifeq ($(CONFIG_PLATFORM_ARM_ROCKCHIP), y)
EXTRA_CFLAGS += -DCONFIG_LITTLE_ENDIAN -DCONFIG_PLATFORM_ANDROID
EXTRA_CFLAGS += -DCONFIG_IOCTL_CFG80211 -DRTW_USE_CFG80211_STA_EVENT
EXTRA_CFLAGS += -DCONFIG_RADIO_WORK
EXTRA_CFLAGS += -DCONFIG_CONCURRENT_MODE
ifeq ($(shell test $(CONFIG_RTW_ANDROID) -ge 11; echo $$?), 0)
EXTRA_CFLAGS += -DCONFIG_IFACE_NUMBER=2
#EXTRA_CFLAGS += -DCONFIG_PLATFORM_ROCKCHIPS
endif

ARCH := arm
CROSS_COMPILE := /home/android_sdk/Rockchip/Rk3188/prebuilts/gcc/linux-x86/arm/arm-eabi-4.6/bin/arm-eabi-
KSRC := /home/android_sdk/Rockchip/Rk3188/kernel

ifeq ($(CONFIG_PCI_HCI), y)
EXTRA_CFLAGS += -DCONFIG_PLATFORM_OPS
_PLATFORM_FILES := platform/platform_linux_pc_pci.o \
		platform/platform_ARM_RK_pci.o

OBJS += $(_PLATFORM_FILES)
# Core Config
# CONFIG_RTKM - n/m/y for not support / standalone / built-in
CONFIG_RTKM = m
EXTRA_CFLAGS += -DCONFIG_TX_SKB_ORPHAN
# PHL Config
EXTRA_CFLAGS += -DRTW_WKARD_98D_RXTAG
endif

ifeq ($(CONFIG_SDIO_HCI), y)
_PLATFORM_FILES = platform/platform_ARM_RK_sdio.o
endif
endif
