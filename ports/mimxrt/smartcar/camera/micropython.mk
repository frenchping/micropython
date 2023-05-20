CAM_MOD_DIR := $(USERMOD_DIR)
SRC_USERMOD += $(addprefix $(CAM_MOD_DIR)/, \
	Camera_py.c \
	modcamera.c \
)
CFLAGS_USERMOD += -I$(CAM_MOD_DIR) -DENABLE_CAMERA_MT9V032