
obj-$(CONFIG_CSI_VFE) += device/
obj-$(CONFIG_CSI_VFE) += actuator/
obj-$(CONFIG_CSI_VFE) += vfe_io.o
vfe_io-y					:= csi_cci/csi_cci_reg.o
vfe_io-y					+= csi_cci/bsp_cci.o
vfe_io-y					+= csi_cci/cci_helper.o
vfe_io-y					+= csi_cci/cci_platform_drv.o
vfe_io-y					+= vfe_subdev.o
vfe_io-y					+= vfe_os.o
vfe_io-y					+= device/sensor_helper.o

ifneq ($(strip $(CONFIG_ARCH_SUN3IW1P1)),)
else
obj-$(CONFIG_CSI_VFE_SAMPLE) += vfe_sample.o
vfe_sample-y					+= sample.o
endif

obj-$(CONFIG_CSI_VFE) += vfe_v4l2.o
vfe_v4l2-y					:= csi/sunxi_csi.o
vfe_v4l2-y					+= mipi_csi/sunxi_mipi.o
vfe_v4l2-y					+= flash_light/flash.o
vfe_v4l2-y					+= sunxi_isp.o

ifneq ($(strip $(CONFIG_ARCH_SUN3IW1P1)),)
vfe_v4l2-y                                      += csi/csi_reg_v1.o
else ifneq ($(strip $(CONFIG_ARCH_SUN8IW11)),)
vfe_v4l2-y					+= csi/csi_reg_v1.o
else
vfe_v4l2-y					+= csi/csi_reg.o
endif
vfe_v4l2-y					+= csi/bsp_csi.o
vfe_v4l2-y					+= bsp_common.o
vfe_v4l2-y					+= config.o
vfe_v4l2-y					+= utility/sensor_info.o
vfe_v4l2-y					+= isp_cfg/isp_cfg.o
vfe_v4l2-y					+= utility/cfg_op.o
vfe_v4l2-y					+= vfe.o


ifneq ($(strip $(CONFIG_CSI_VFE)), )
ifneq ($(strip $(CONFIG_ARCH_SUN3IW1P1)),)
vfe_v4l2-y					+= lib/bsp_isp_null.o
else ifneq ($(strip $(CONFIG_ARCH_SUN8IW10)),)
vfe_v4l2-y                                      += lib/bsp_isp_null.o
else ifneq ($(strip $(CONFIG_ARCH_SUN8IW11)),)
vfe_v4l2-y					+= lib/bsp_isp_null.o
else
ifdef CONFIG_ARM
$(shell cp $(obj)/lib/libisp_32 $(obj)/lib/libisp_32.a)
vfe_v4l2-y					+= lib/libisp_32.a
endif
ifdef CONFIG_ARM64
$(shell cp $(obj)/lib/libisp_64 $(obj)/lib/libisp_64.a)
vfe_v4l2-y					+= lib/libisp_64.a
endif
endif
endif

ifneq ($(strip $(CONFIG_ARCH_SUN9I)),)
vfe_v4l2-y					+= lib/lib_mipicsi2_v2
else ifneq ($(strip $(CONFIG_ARCH_SUN8IW6)),)
obj-$(CONFIG_CSI_VFE) += vfe_v4l2.o
vfe_v4l2-y					+= mipi_csi/bsp_mipi_csi.o
else ifneq ($(strip $(CONFIG_ARCH_SUN8IW8)),)
obj-$(CONFIG_CSI_VFE) += vfe_v4l2.o
vfe_v4l2-y					+= mipi_csi/bsp_mipi_csi_v1.o \
							mipi_csi/protocol/protocol_reg.o \
							mipi_csi/dphy/dphy_reg.o
else
obj-$(CONFIG_CSI_VFE) += vfe_v4l2.o
vfe_v4l2-y					+= mipi_csi/bsp_mipi_csi_null.o
endif
