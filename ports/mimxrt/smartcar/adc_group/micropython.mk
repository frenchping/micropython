ADCG_MOD_DIR := $(USERMOD_DIR)
SRC_USERMOD += $(addprefix $(ADCG_MOD_DIR)/, \
	adc_grp_py.c \
	adc_drv.c \
	modsensor.c \
)
CFLAGS_USERMOD += -I$(ADCG_MOD_DIR) -I$(ADCG_MOD_DIR)/../include
