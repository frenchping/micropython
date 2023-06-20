DISP_MOD_DIR := $(USERMOD_DIR)
SRC_USERMOD += $(addprefix $(DISP_MOD_DIR)/, \
	LCD_py.c \
	LCD_Driver.c \
	Terminal.c \
	lcd_spi_drv.c \
	lcd_bg_print.c \
	moddisplay.c \
	lcd_fonts.c \
	font_py.c \
)
CFLAGS_USERMOD += -I$(DISP_MOD_DIR)