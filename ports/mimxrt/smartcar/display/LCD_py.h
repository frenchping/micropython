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

#include "lcd_spi_drv.h"
#include "LCD_Drvier.h"

#ifndef _INCLUDE_LCD_PY_H
#define _INCLUDE_LCD_PY_H

extern const mp_obj_type_t display_lcd_type;

extern const mp_obj_fun_builtin_var_t       lcd_clear_obj;
extern const mp_obj_fun_builtin_fixed_t     lcd_color_obj;
extern const mp_obj_fun_builtin_fixed_t     lcd_mode_obj;

typedef union _color_values {
    uint16_t color;
    struct _rgb_ {
        uint16_t b: 5;
        uint16_t g: 6;
        uint16_t r: 5;
    } RGB;
} color_value_t;

typedef struct _display_string_t {
    uint16_t    loc_x;
    uint16_t    loc_y;
    char       *str;
    uint16_t    length;
    uint16_t    color;
} display_string_t;

typedef struct _display_lcd_obj_t {
    mp_obj_base_t base;
    lcd_drv_obj_t *lcd_drv;
} display_lcd_obj_t;

void lcd_str_screen(lcd_display_t *lcd, display_string_t *dstr, const Asc_Font_t *fontp);

/** The protocol interface **/
typedef struct _display_lcd_p_t {
    void (*blit)(display_lcd_obj_t *lcd_obj, Rect_t *rect, mp_buffer_info_t *buf);
} display_lcd_p_t;

#endif  // _INCLUDE_LCD_PY_H
