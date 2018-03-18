dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb
# For 8173 projects
ifneq ($(CONFIG_TB8173_P1), )
ARCH_MTK_PROJECT_DTB := $(subst ",,$(CONFIG_ARCH_MTK_PROJECT))
dtb-$(CONFIG_ARCH_MT8173) += $(ARCH_MTK_PROJECT_DTB).dtb
else
ifneq ($(CONFIG_BX8173_P12), )
dtb-$(CONFIG_ARCH_MT8173) += bx8173p12.dtb
else
dtb-$(CONFIG_ARCH_MT8173) += tb8173evb.dtb
dtb-$(CONFIG_ARCH_MT8173) += tb8173p1.dtb
dtb-$(CONFIG_ARCH_MT8173) += tb8173p1_therm.dtb
dtb-$(CONFIG_ARCH_MT8173) += tb8173fpga.dtb
dtb-$(CONFIG_ARCH_MT8173) += bx8173p1.dtb
dtb-$(CONFIG_ARCH_MT8173) += tb8173p1_gpu.dtb
dtb-$(CONFIG_ARCH_MT8173) += bx8173p12.dts
endif
endif
targets += dtbs

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif
targets += $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := dts/*.dtb *.dtb
