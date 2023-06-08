SCAR_MOD_DIR := $(USERMOD_DIR)
SRC_USERMOD += $(addprefix $(SCAR_MOD_DIR)/, \
	adc_drv.c \
	adc_grp_py.c \
	Camera_py.c \
	mod_scar.c \
	Encoder_py.c \
)
CFLAGS_USERMOD += -I$(SCAR_MOD_DIR) -DENABLE_CAMERA_MT9V032 -I$(SCAR_MOD_DIR)/../include