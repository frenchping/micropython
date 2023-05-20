/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Damien P. George
 * Copyright (c) 2020 Jim Mussared
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"

#include "pin.h"

#include CLOCK_CONFIG_H
#include "fsl_iomuxc.h"
#include "fsl_lpspi.h"

#include "lcd_spi_drv.h"
#include "LCD_Drvier.h"
#include "LCD_Drv_Help.h"

// Remember the effective drv. It will be used by LCD class.
lcd_drv_obj_t *current_drv;

/// @brief find_pin_by_mux() is used to get pin_obj of CS pin
/// It will be easy if machine_spi.c explorers its machine_spi_obj_t.
typedef struct _iomux_table_t {
    uint32_t muxRegister;
    uint32_t muxMode;
    uint32_t inputRegister;
    uint32_t inputDaisy;
    uint32_t configRegister;
} iomux_table_t;

#define LCD_SPI_BAUDRATE    6000000
#define LCD_SPI_POLARITY    kLPSPI_ClockPolarityActiveLow
#define LCD_SPI_PHASE       kLPSPI_ClockPhaseSecondEdge

STATIC mp_obj_t lcd_drv_help(size_t n_args, const mp_obj_t *args) {
    if (n_args == 0)
        mp_print_str(MP_PYTHON_PRINTER, LCD_DRV_HELP_TEXT);
    else {
        int help_id = mp_obj_get_int(args[0]);
        char *help_text[LCD_DRV_LAST_HELP] = {
            LCD_DRV_NEW_HELP_TEXT,
            LCD_DRV_INIT_HELP_TEXT,
            LCD_DRV_MODE_HELP_TEXT,
            LCD_DRV_AREA_HELP_TEXT,
            LCD_DRV_CMD_HELP_TEXT,
            LCD_DRV_DATA_HELP_TEXT
        };
        if (help_id >= 0 && help_id < LCD_DRV_LAST_HELP)
            mp_print_str(MP_PYTHON_PRINTER, help_text[help_id]);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_drv_help_func, 0, 1, lcd_drv_help);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(lcd_drv_help_obj, MP_ROM_PTR(&lcd_drv_help_func));

const machine_pin_obj_t *find_pin_by_mux(uint32_t muxRegister)
{
    for (uint32_t ipin = 0; ipin < num_board_pins; ipin++) {
        if (muxRegister == machine_pin_board_pins[ipin]->muxRegister)
            return machine_pin_board_pins[ipin];
    }
    return NULL;
}

void lcd_drv_init_spi(LPSPI_Type *spi, const iomux_table_t *pins, int rate)
{
    lpspi_master_config_t   master_config;

    LPSPI_Deinit(spi);      // Disable it first

    LPSPI_MasterGetDefaultConfig(&master_config);
    // Initialise the SPI peripheral.
    master_config.baudRate = rate;
    master_config.cpol = LCD_SPI_POLARITY;
    master_config.cpha = LCD_SPI_PHASE;
    master_config.pcsToSckDelayInNanoSec    = 5;
    master_config.lastSckToPcsDelayInNanoSec= 5;
    master_config.betweenTransferDelayInNanoSec = 5;
    LPSPI_MasterInit(spi, &master_config, BOARD_BOOTCLOCKRUN_LPSPI_CLK_ROOT);

    uint32_t pad_ctrl = IOMUXC_SW_PAD_CTL_PAD_PUS(0b01) | IOMUXC_SW_PAD_CTL_PAD_PUE(1) |
                        IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(0b100) |
                        IOMUXC_SW_PAD_CTL_PAD_SPEED(0b10);
    // SCK
    IOMUXC_SetPinMux(pins->muxRegister, pins->muxMode, pins->inputRegister, pins->inputDaisy, pins->configRegister, 0U);
    IOMUXC_SetPinConfig(pins->muxRegister, pins->muxMode, pins->inputRegister, pins->inputDaisy, pins->configRegister, pad_ctrl);
    // PCS0
    pins++;
    IOMUXC_SetPinMux(pins->muxRegister, pins->muxMode, pins->inputRegister, pins->inputDaisy, pins->configRegister, 0U);
    IOMUXC_SetPinConfig(pins->muxRegister, pins->muxMode, pins->inputRegister, pins->inputDaisy, pins->configRegister, pad_ctrl);
    // SDO
    pins++;
    IOMUXC_SetPinMux(pins->muxRegister, pins->muxMode, pins->inputRegister, pins->inputDaisy, pins->configRegister, 0U);
    IOMUXC_SetPinConfig(pins->muxRegister, pins->muxMode, pins->inputRegister, pins->inputDaisy, pins->configRegister, pad_ctrl);
}

/// \classmethod \constructor()
/// Create and return an LCD_Drv object.
///
/// User has to derive this class with following constants.
/// Some have default values:
///     SPI_INDEX = must provide
///     DC_PIN  =   must provide
///     RST_PIN =   should provide, can be missing if panel need no RESET
///     LCD_WIDTH = 0   must provide for user-defined
///     LCD_HEIGHT= 0   must provide for user-defined
///     COLOR_DEPTH = 16
///     LCD_TYPE = -1   User defined type. Some pre-defined type exist.
STATIC mp_obj_t lcd_drv_obj_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args_in)
{
    mp_arg_check_num(n_args, n_kw, 0, 7, true);

    enum { ARG_spi, ARG_dc, ARG_rst, ARG_baudrate, ARG_width, ARG_height, ARG_depth, ARG_type };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_SPI_INDEX,    MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_DC_PIN,       MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_RST_PIN,      MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = mp_const_none}},

        { MP_QSTR_BAUDRATE,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = LCD_SPI_BAUDRATE} },
        { MP_QSTR_LCD_WIDTH,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_LCD_HEIGHT,   MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_COLOR_DEPTH,  MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 16}},
        { MP_QSTR_LCD_TYPE,     MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1}},
    };
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args_in + n_args);

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, args_in, &kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    LPSPI_Type *spi_base;       // The SPI bus
    const machine_pin_obj_t *cs_pin = NULL;
    int spi_id = args[ARG_spi].u_int;   // Mpy id
    
        // Get the HW index of SPI
        const uint8_t spi_index_table[] = MICROPY_HW_SPI_INDEX;
        uint8_t spi_hw_id = spi_index_table[spi_id];        // the hw spi number 1..n
        if (spi_hw_id < 1 || spi_hw_id > FSL_FEATURE_SOC_LPSPI_COUNT)
            mp_raise_ValueError(MP_ERROR_TEXT("Invalid SPI_INDEX"));

        LPSPI_Type *spi_base_ptr_table[] = LPSPI_BASE_PTRS;
        spi_base = spi_base_ptr_table[spi_hw_id];
    
        spi_hw_id --;
        const iomux_table_t iomux_table[] = { IOMUX_TABLE_SPI };
        const iomux_table_t *spi_pins = &iomux_table[spi_hw_id*5];

        cs_pin = find_pin_by_mux(spi_pins[1].muxRegister);
        if (cs_pin == NULL)
            mp_raise_ValueError(MP_ERROR_TEXT("Invalid SPI_INDEX"));

        lcd_drv_init_spi(spi_base, spi_pins, args[ARG_baudrate].u_int);
    
    lcd_drv_obj_t *self = mp_obj_malloc(lcd_drv_obj_t, &display_lcd_drv_type);
    current_drv = self;     // LCD class will use it directly

    self->lcd.lpspi_id = spi_hw_id + 1;
    self->lcd.dc_pin = (const machine_pin_obj_t *)(args[ARG_dc].u_obj);
    self->lcd.rst_pin = (const machine_pin_obj_t *)(args[ARG_rst].u_obj);
    self->lcd.sck_pin = find_pin_by_mux(spi_pins[0].muxRegister);
    self->lcd.sdo_pin = find_pin_by_mux(spi_pins[2].muxRegister);

    self->lcd.cs_pin  = cs_pin;
    self->lcd.spi_base = spi_base;
    memset(&self->lcd.user_ops, 0, sizeof(lcd_user_drv_t)); // Clear the operation table

    self->lcd.lcd_type = args[ARG_type].u_int;
    self->lcd.Screen_Width = args[ARG_width].u_int;
    self->lcd.Screen_Height= args[ARG_height].u_int;
    self->lcd.Color_Depth  = args[ARG_depth].u_int;

    if (self->lcd.lcd_type > SPI_LCD_NUM_TYPE)
        if (self->lcd.Screen_Width == 0 || self->lcd.Screen_Height == 0)
            mp_raise_msg_varg(&mp_type_ValueError,
                    MP_ERROR_TEXT("Neither LCD_WIDTH=%d nor LCD_HEIGHT=%d can be 0."),
                    self->lcd.Screen_Width, self->lcd.Screen_Height);

    return MP_OBJ_FROM_PTR(self);
}

STATIC void lcd_drv_obj_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    (void)kind;
    lcd_drv_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "    LCD_Drv (spi_base=0x%X, LPSPI%d)\r\n", self->lcd.spi_base, self->lcd.lpspi_id);
    mp_printf(print, "\tDC Pin  = %s\r\n", qstr_str(self->lcd.dc_pin->name));
    mp_printf(print, "\tRST Pin = %s\r\n", qstr_str(self->lcd.rst_pin->name));
    mp_printf(print, "\tCS Pin  = %s\r\n", qstr_str(self->lcd.cs_pin->name));
    mp_printf(print, "\tSCK Pin = %s\r\n", qstr_str(self->lcd.sck_pin->name));
    mp_printf(print, "\tSDO Pin = %s\r\n", qstr_str(self->lcd.sdo_pin->name));

    mp_printf(print, "\tLCD_Type = %d; Orientation mode = %d\r\n", self->lcd.lcd_type, self->lcd.Orientation);
    mp_printf(print, "\tScreen (Width = %d, Height = %d)\r\n", self->lcd.Screen_Width, self->lcd.Screen_Height);
}

/// @brief This should be overriden by user for self-defined type.
///         This is provide here to remind user to override it
STATIC mp_obj_t lcd_drv_init(mp_obj_t self_in)
{
    mp_raise_NotImplementedError(MP_ERROR_TEXT("Has to define init() for user-defined screen type."));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(lcd_drv_init_obj, lcd_drv_init);

/// @brief If user override init(), this is the bridge to call user function from lower layer
/* STATIC */int user_init(mp_obj_t user_self)
{
    mp_obj_t dest[2];
    mp_load_method_maybe(user_self, MP_QSTR_init, dest);  // Get USER func address
    /*mp_obj_t ret = */mp_call_function_1(dest[0], dest[1]);
    return 0;
}

/// @brief Set the display mode. 90°, 180°, 270° etc.
///         This is provide here to remind user to override it
STATIC mp_obj_t lcd_drv_mode(mp_obj_t self_in, mp_obj_t mode_in)
{
    mp_raise_NotImplementedError(MP_ERROR_TEXT("Has to define set_mode(mode) for user-defined screen type."));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(lcd_drv_mode_obj, lcd_drv_mode);

/// @brief If user override set_mode(mode), this is the bridge to call user function from lower layer
/* STATIC */int user_mode(mp_obj_t user_self, int mode)
{
    mp_obj_t dest[2];
    mp_load_method_maybe(user_self, MP_QSTR_set_mode, dest);  // Get USER func address
    mp_obj_t arg = MP_OBJ_NEW_SMALL_INT(mode);
    /*mp_obj_t ret = */mp_call_function_2(dest[0], dest[1], arg);
    return 0;
}

STATIC int user_area(mp_obj_t user_self, Rect_t *rect)
{
    // extern void fool_stack_check(void);
    // extern void nofool_stack_check(void);

    mp_obj_t dest[2];
    mp_load_method_maybe(user_self, MP_QSTR_set_area, dest);  // Get USER func address
    mp_obj_t args[5];
    args[0] = dest[1];
    args[1] = MP_OBJ_NEW_SMALL_INT(rect->x0);
    args[2] = MP_OBJ_NEW_SMALL_INT(rect->y0);
    args[3] = MP_OBJ_NEW_SMALL_INT(rect->width);
    args[4] = MP_OBJ_NEW_SMALL_INT(rect->height);
        // thread_fool_stack_check();
    /*mp_obj_t ret = */mp_call_function_n_kw(dest[0], 5, 0, args);
        // thread_nofool_stack_check();
    return 0;
}

/// @brief Write a series of bytes to LCD.
/// @param self_in : LCD object
/// @param cmds : the bytearray
/// @return 
STATIC mp_obj_t lcd_drv_write_cmd(mp_obj_t self_in, mp_obj_t cmds)
{
    lcd_drv_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(cmds, &bufinfo, MP_BUFFER_READ);

    spi_lcd_write_cmd(&self->lcd, bufinfo.buf, bufinfo.len-1);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(lcd_drv_write_cmd_obj, lcd_drv_write_cmd);

/// @brief Write a series of (uint16_t) to the LCD. Used to send pixel data
/// @param self_in : LCD object
/// @param data_in : the bytearray
/// @return 
STATIC mp_obj_t lcd_drv_write_data(mp_obj_t self_in, mp_obj_t data_in)
{
    lcd_drv_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_in, &bufinfo, MP_BUFFER_READ);
//// TO BE FIXED
    spi_lcd_write_data(&self->lcd, *(uint16_t*)bufinfo.buf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(lcd_drv_write_data_obj, lcd_drv_write_data);

/// @brief This is provided for user-defined panel. It will not be called from lower layer
///         If user override this function, the lower layer will use user-provided set_area()
STATIC mp_obj_t lcd_drv_set_area(size_t n_args, const mp_obj_t *args)
{
    lcd_drv_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    Rect_t  rect;
    rect.x0     = mp_obj_get_int(args[1]);
    rect.y0     = mp_obj_get_int(args[2]);
    rect.width  = mp_obj_get_int(args[3]);
    rect.height = mp_obj_get_int(args[4]);
    spi_lcd_set_window(&self->lcd, &rect);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_drv_set_area_obj, 5, 5, lcd_drv_set_area);

STATIC const mp_rom_map_elem_t lcd_drv_locals_dict_table[] = {
    // These functions can be overridden for user-defined panel.
    { MP_ROM_QSTR(MP_QSTR_help),        MP_ROM_PTR(&lcd_drv_help_obj) },

    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&lcd_drv_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_mode),    MP_ROM_PTR(&lcd_drv_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_area),    MP_ROM_PTR(&lcd_drv_set_area_obj) },

    // These functions are primary operation and can not be overridden.
    { MP_ROM_QSTR(MP_QSTR_write_cmd),   MP_ROM_PTR(&lcd_drv_write_cmd_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_data),  MP_ROM_PTR(&lcd_drv_write_data_obj) },
    // { MP_ROM_QSTR(MP_QSTR_fill_color),  MP_ROM_PTR(&lcd_drv_fill_color_obj) },

    // There are a few pre-defined panel.
    // { MP_ROM_QSTR(MP_QSTR_LCD180_TYPE),  MP_ROM_INT(SPI_LCD_180) },
    // { MP_ROM_QSTR(MP_QSTR_LCD350_TYPE),  MP_ROM_INT(SPI_LCD_350) },
    // { MP_ROM_QSTR(MP_QSTR_LCD154_TYPE),  MP_ROM_INT(SPI_LCD_154) },
    { MP_ROM_QSTR(MP_QSTR_LCD200_TYPE),  MP_ROM_INT(SPI_LCD_200) },
};
STATIC MP_DEFINE_CONST_DICT(lcd_drv_locals_dict, lcd_drv_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    display_lcd_drv_type,
    MP_QSTR_LCD_Drv,
    MP_TYPE_FLAG_NONE,
    make_new, lcd_drv_obj_make_new,
    print, lcd_drv_obj_print,
    locals_dict, &lcd_drv_locals_dict
    );



/*****************************************************************************
 * Function called by LCD_py class
 * 
 *****************************************************************************/
typedef struct _lcd_drv_name_fun_t {
    qstr    name;
    void*   func;
} lcd_drv_func_table_t;

#define FUNC_TABLE_ITEM(n, f)    { .name = n, .func = f }
STATIC const lcd_drv_func_table_t func_table[] = {
    FUNC_TABLE_ITEM(MP_QSTR_init,       user_init),
    FUNC_TABLE_ITEM(MP_QSTR_set_mode,   user_mode),
    FUNC_TABLE_ITEM(MP_QSTR_set_area,   user_area),
    FUNC_TABLE_ITEM(MP_QSTRnull,   NULL),
};

/// @brief Check if functions in LCD_Drv are overridden by user.
///         If so, remember them and later lower layer will use the new functions.
/// @param user_drv_obj : The object of possible derived LCD_Drv
/// @param lcd_dev      : The lcd device
void check_override_func(mp_obj_t user_drv_obj, lcd_display_t *lcd)
{
    int ifun;
    mp_obj_t dest_usr[2];
    lcd_user_drv_t override_func;
    memset(&override_func, 0, sizeof(override_func));
    for (ifun = 0; func_table[ifun].name != MP_QSTRnull; ifun++) {
        mp_load_method_maybe(user_drv_obj, func_table[ifun].name, dest_usr);  // Get USER func address
        // It is sure that dest[0] in not NULL. Because I have the default implementation.
        switch (func_table[ifun].name) {
            case MP_QSTR_init:
                override_func.screen_init = func_table[ifun].func;
                break;
            case MP_QSTR_set_mode:
                override_func.set_mode = func_table[ifun].func;
                break;
            case MP_QSTR_set_area:
                // Do not set the default implementation. It is taking care in lower layer.
                if (dest_usr[0] != MP_ROM_PTR(&lcd_drv_set_area_obj))
                    override_func.set_window = func_table[ifun].func;
                break;
            default:
                break;
        }
        override_func.user_obj = user_drv_obj;
    }
    if (override_func.user_obj) {
        lcd->user_ops.screen_init = override_func.screen_init;
        lcd->user_ops.set_mode    = override_func.set_mode;
        lcd->user_ops.set_window  = override_func.set_window;
        lcd->user_ops.user_obj = user_drv_obj;
    }
}
