ccflags-$(CONFIG_PROSLIC)	+=-Idrivers/slic/proslic_api/inc
ccflags-$(CONFIG_PROSLIC)	+=-Idrivers/slic/proslic_api/config_inc
ccflags-$(CONFIG_SLIC_DEBUG)	+=-DDEBUG

PROSLIC_OBJ = \
			proslic_api/src/si_voice.o \
			proslic_api/src/si_voice_version.o \
			proslic_api/src/proslic.o	\
			proslic_api/src/si3217x_intf.o	\
			proslic_api/src/si3217x_revb_intf.o	\
			proslic_api/src/si3217x_revc_intf.o	\
			proslic_api/config_src/si3217x_constants.o \
			proslic_api/patch_files/si3217x_patch_B_BB_2012DEC10.o \
			proslic_api/patch_files/si3217x_patch_B_FB_2012DEC10.o \
			proslic_api/patch_files/si3217x_patch_C_FB_2014JUN18.o \
			proslic_api/src/vdaa.o \
			proslic_api/config_src/vdaa_constants.o

obj-$(CONFIG_SLIC) += slic.o

proslic-objs := si3217x.o proslic_tone.o proslic_config.o $(PROSLIC_OBJ)
obj-$(CONFIG_PROSLIC_SI3217X) += proslic.o
