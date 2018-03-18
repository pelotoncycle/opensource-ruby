include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y += lens.o

ifeq ($(CONFIG_MTK_LENS_DUMMYLENS_SUPPORT),y)
ccflags-y += -DDUMMY_LENS
lens-y += dummy_lens/dummy_lens.o
endif

ifeq ($(CONFIG_MTK_LENS_DW9814AF_SUPPORT),y)
ccflags-y += -DDW9814AF
lens-y += dw9814af/DW9814AF.o
endif
