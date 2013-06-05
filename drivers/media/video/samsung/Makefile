obj-$(CONFIG_VIDEO_FIMC)	+= fimc/
obj-$(CONFIG_VIDEO_JPEG)	+= jpeg/
obj-$(CONFIG_VIDEO_TVOUT)	+= tvout/
obj-$(CONFIG_VIDEO_MFC5X)	+= mfc5x/

obj-$(CONFIG_VIDEO_FIMG2D3X)	+= fimg2d3x-exynos4/

ifeq ($(CONFIG_MACH_U1), y)
obj-$(CONFIG_VIDEO_MALI400MP)	+= ump/
obj-$(CONFIG_VIDEO_MALI400MP)	+= mali/
obj-$(CONFIG_MALI_R3P0_LSI)	+= mali_r3p0_lsi/
endif

obj-$(CONFIG_VIDEO_TSI)		+= tsi/

EXTRA_CFLAGS += -Idrivers/media/video
