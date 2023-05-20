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
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"

#include "lcd_spi_drv.h"
#include "LCD_py.h"
#include "Term_help.h"

extern const mp_obj_type_t display_tty_type;

typedef struct _terminal_obj_t {
    mp_obj_base_t       base;
    display_lcd_obj_t   *lcd_obj;

    uint16_t    tty_x;
    uint16_t    tty_y;
    char        *tty_buf;
    const Asc_Font_t *tty_font;
    uint8_t     mode;
    uint8_t     font;
} terminal_obj_t;

STATIC mp_obj_t tty_help(size_t n_args, const mp_obj_t *args) {
    if (n_args == 0)
        mp_print_str(MP_PYTHON_PRINTER, TTY_HELP_TEXT);
    // else {
    //     int help_id = mp_obj_get_int(args[0]);
    //     char *help_text[LCD_LAST_HELP] = {
    //         LCD_NEW_HELP_TEXT,
    //         LCD_STR_HELP_TEXT,
    //         LCD_CLEAR_HELP_TEXT,
    //         LCD_MODE_HELP_TEXT,
    //         LCD_COLOR_HELP_TEXT,
    //         LCD_LINE_HELP_TEXT,
    //         LCD_BLIT_HELP_TEXT
    //     };
    //     if (help_id >= 0 && help_id < LCD_LAST_HELP)
    //         mp_print_str(MP_PYTHON_PRINTER, help_text[help_id]);
    // }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tty_help_func, 0, 1, tty_help);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(tty_help_obj, MP_ROM_PTR(&tty_help_func));

STATIC void tty_obj_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    terminal_obj_t *self = self_in;
    lcd_display_t *lcd = &self->lcd_obj->lcd_drv->lcd;
    mp_printf(print, "    ---- Terminal Object ----\r\n");
    mp_printf(print, "\tWidth = %d, Height = %d\r\n", lcd->Width, lcd->Height);
    mp_printf(print, "\tCursor = (%d, %d); Font = %d\r\n", self->tty_x, self->tty_y, self->font);
    mp_printf(print, "\tLCD_Type = %d; Orientation mode = %d\r\n", lcd->lcd_type, lcd->Orientation);
}

/// \classmethod \constructor()
///
/// Terminal(LCD object)
STATIC mp_obj_t tty_obj_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    mp_obj_type_t *obj_in = (mp_obj_type_t*)MP_OBJ_TO_PTR(args[0]);
    const mp_obj_type_t *usr_drv_type = obj_in->base.type;

    if (!mp_obj_is_subclass_fast(usr_drv_type, MP_OBJ_FROM_PTR(&display_lcd_type)))
        mp_raise_TypeError(MP_ERROR_TEXT("Provided object has to be LCD or derived from LCD."));

    display_lcd_obj_t *lcd_obj = (display_lcd_obj_t*)MP_OBJ_TO_PTR(args[0]);

    terminal_obj_t *self = mp_obj_malloc(terminal_obj_t, &display_tty_type);
    self->lcd_obj = lcd_obj;

    // Initialize all terminal mode variables.
    // Default is no tty function
    self->tty_x = 0;
    self->tty_y = 0;
    self->tty_font = 0;
    self->tty_buf = NULL;
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t tty_clear(size_t n_args, const mp_obj_t *args)
{
    terminal_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    display_lcd_obj_t *lcd_obj = self->lcd_obj;

    // call LCD.clear()
    mp_obj_t dest[3] = { (mp_obj_t)&lcd_clear_obj, (mp_obj_t)lcd_obj, 0 };
    if (n_args == 1)
        mp_call_method_n_kw(0, 0, dest);
    else {
        dest[2] = args[1];
        mp_call_method_n_kw(1, 0, dest);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tty_clear_obj, 1, 2, tty_clear);

STATIC mp_obj_t tty_mode(mp_obj_t self_in, mp_obj_t mode_in)
{
    terminal_obj_t *self = MP_OBJ_TO_PTR(self_in);
    display_lcd_obj_t *lcd_obj = self->lcd_obj;
    
    mp_obj_t dest[3] = { (mp_obj_t)&lcd_mode_obj, (mp_obj_t)lcd_obj, mode_in };
    mp_call_method_n_kw(1, 0, dest);

    self->mode = mp_obj_get_int(mode_in);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(tty_mode_obj, tty_mode);

STATIC mp_obj_t tty_color(mp_obj_t self_in, mp_obj_t fg_in, mp_obj_t bg_in)
{
    terminal_obj_t *self = MP_OBJ_TO_PTR(self_in);
    display_lcd_obj_t *lcd_obj = self->lcd_obj;

    mp_obj_t dest[4] = { (mp_obj_t)&lcd_color_obj, (mp_obj_t)lcd_obj, fg_in, bg_in };
    mp_call_method_n_kw(2, 0, dest);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(tty_color_obj, tty_color);

STATIC mp_obj_t tty_clear_line(mp_obj_t self_in, mp_obj_t font_in)
{
    // terminal_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(tty_clear_line_obj, tty_clear_line);

/// @brief Set the font used in terminal mode
/// @return 
STATIC mp_obj_t tty_font(mp_obj_t self_in, mp_obj_t font_in)
{
    terminal_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int font = mp_obj_get_int(font_in);
    if (font == 12)
        self->tty_font = &Asc_Font12;
    else if (font == 16)
        self->tty_font = &Asc_Font16;
    else if (font == 24)
        self->tty_font = &Asc_Font24;
    else if (font == 32)
        self->tty_font = &Asc_Font32;
    else
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid fontsize value."));
    self->font = font;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(tty_font_obj, tty_font);

/// @brief Set the display cursor in terminal mode
/// @return 
STATIC mp_obj_t tty_cursor(size_t n_args, const mp_obj_t *args)
{
    terminal_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args == 1)
        self->tty_x = self->tty_y = 0;
    else {
        self->tty_x = mp_obj_get_int(args[1]);
        if (n_args == 3) 
            self->tty_y = mp_obj_get_int(args[2]);
        else
            self->tty_y = 0;
    } 
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(tty_cursor_obj, 1, 3, tty_cursor);

STATIC const mp_rom_map_elem_t tty_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_help),    MP_ROM_PTR(&tty_help_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),   MP_ROM_PTR(&tty_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_color),   MP_ROM_PTR(&tty_color_obj) },
    { MP_ROM_QSTR(MP_QSTR_mode),    MP_ROM_PTR(&tty_mode_obj) },

    { MP_ROM_QSTR(MP_QSTR_clear_line),    MP_ROM_PTR(&tty_clear_line_obj) },

    { MP_ROM_QSTR(MP_QSTR_font),     MP_ROM_PTR(&tty_font_obj) },
    { MP_ROM_QSTR(MP_QSTR_cursor),   MP_ROM_PTR(&tty_cursor_obj) },
};
STATIC MP_DEFINE_CONST_DICT(tty_locals_dict, tty_locals_dict_table);

/****************************************************************
 * Functions for protocol
*/
#include "py/stream.h"

mp_uint_t tty_stream_write(mp_obj_t self_in, const void *buf_in, mp_uint_t size, int *errcode)
{
    terminal_obj_t *self = self_in;
    lcd_display_t *lcd = &self->lcd_obj->lcd_drv->lcd;
    display_string_t dstr;
    char *string = (char*)buf_in;
    int  remain = size;

    int xchar = lcd->Width / self->tty_font->Width;  // number of char's in one line
    int ychar = lcd->Height/ self->tty_font->Height; // number of lines

    while (remain) {
        if (self->tty_x >= xchar) {
            self->tty_x = 0;
            self->tty_y++;
        }
        if (self->tty_y >= ychar)
            self->tty_y = 0;     // Reset it when reach to screen end

        dstr.color = lcd->Color_Foreground;
        dstr.loc_x = self->tty_x * self->tty_font->Width;
        dstr.loc_y = self->tty_y * self->tty_font->Height;
        dstr.str   = string;
        dstr.length= xchar - self->tty_x;
        if (dstr.length > remain)
            dstr.length = remain;
        remain -= dstr.length;
        lcd_str_screen(lcd, &dstr, self->tty_font);

        self->tty_x += dstr.length;
        string += dstr.length;
    }
    return size;
}

STATIC const mp_stream_p_t display_stream_p = {
    // .read = tty_stream_read,
    .write = tty_stream_write,
    // .ioctl = tty_stream_ioctl,
    .is_text = false,
};

MP_DEFINE_CONST_OBJ_TYPE(
    display_tty_type,
    MP_QSTR_Terminal,
    MP_TYPE_FLAG_NONE,
    make_new, tty_obj_make_new,
    print, tty_obj_print,
    protocol, &display_stream_p,
    locals_dict, &tty_locals_dict
    );
