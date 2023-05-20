
# MPY_DIR = ../../
# MIMRX_DIR = ./

SDSPI_MOD_DIR := $(USERMOD_DIR)
SRC_USERMOD += $(addprefix $(SDSPI_MOD_DIR)/, \
	fsl_sdspi.c \
	sdspi_py.c \
	modsdspi.c \
)

CFLAGS_USERMOD += -I$(SDSPI_MOD_DIR) 
