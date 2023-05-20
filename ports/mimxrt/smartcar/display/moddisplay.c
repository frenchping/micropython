/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "py/runtime.h"

extern const mp_obj_type_t display_lcd_type;
extern const mp_obj_type_t display_lcd_drv_type;
extern const mp_obj_type_t display_tty_type;
extern const mp_obj_module_t mp_module_font;

extern const mp_obj_module_t camera_type;

STATIC const mp_rom_map_elem_t display_module_globals_table[] = {
    // { MP_ROM_QSTR(MP_QSTR___name__),            MP_ROM_QSTR(MP_QSTR_display) },
    { MP_ROM_QSTR(MP_QSTR_font),            MP_ROM_PTR(&mp_module_font) },
    { MP_ROM_QSTR(MP_QSTR_LCD),             MP_ROM_PTR(&display_lcd_type) },
    { MP_ROM_QSTR(MP_QSTR_LCD_Drv),         MP_ROM_PTR(&display_lcd_drv_type) },
    { MP_ROM_QSTR(MP_QSTR_Terminal),        MP_ROM_PTR(&display_tty_type) },

    // { MP_ROM_QSTR(MP_QSTR_MT9V032), MP_ROM_PTR(&camera_type) },
};
STATIC MP_DEFINE_CONST_DICT(display_module_globals, display_module_globals_table);

const mp_obj_module_t mp_display_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&display_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_display, mp_display_module);
