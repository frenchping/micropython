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

#ifndef DEFINE_LCD_HELP_H
#define DEFINE_LCD_HELP_H


typedef enum _lcd_help_id {
    LCD_NEW_HELP = 0,   // Constructor
    LCD_STR_HELP,       // str??()
    LCD_CLEAR_HELP,     // clear()
    LCD_MODE_HELP,      // mode()
    LCD_COLOR_HELP,     // color()
    LCD_LINE_HELP,      // line()
    LCD_BLIT_HELP,      // blit()
    LCD_LAST_HELP
} lcd_help_t;

#define LCD_HELP_TEXT  \
    "LCD class is defined to generate the raster data to be send to the display panel.\n" \
    "The panel is represents by a LCD_Drv object or an object of derived LCD_Drv class.\n" \
    "-------------------------------\n" \
    "Following methods(functions) are in LCD class:\n"\
    " Constructor:\n" \
    "   0. class display.LCD(drv)   'drv' is a LCD_Drv object.\n" \
    " Methods:\n" \
    "   1. LCD.str##(x, y, text [, color, framebuf])\n" \
    "           Output the 'text' to location (x, y) with optional color.\n" \
    "           Default color is defined by LCD.color().\n" \
    "           If parameter 'framebuf' is given, the text outputs to framebuf object.\n"\
    "           Four font sizes are defined:\n" \
    "                   6x12(str12), 8x16(str16), 12x24(str24) & 16x32(str32).\n" \
    "   2. LCD.clear(color)\n"\
    "           Fill the screen with given 'color'. If parameter 'color' is missing,\n" \
    "           use background color defined by LCD.color().\n" \
    "   3. LCD.mode(mode)\n" \
    "           Set the screen orientation (See LCD_Drv).\n" \
    "   4. LCD.color(fg, bg)\n" \
    "           Defined the drawing color of foreground and background.\n" \
    "   5. LCD.line(x0, y0, x1, y1, *, color, thick=1)\n" \
    "           Draw a line from (x0, y0) to (x1, y1).\n" \
    "           The line color is given by 'color' or forecolor defined by LCD.color()\n" \
    "           The line width is given by 'thick' or default as 1 pixel.\n" \
    "   6. LCD.blit(x, y, w, h, buffer)\n" \
    "           Copy pixels in the buffer to the screen location defined by x & y.\n" \
    "           The 'buffer' provide a rectangle bitmap data and w & h provide the width\n" \
    "           and height of the rectangle.\n" \
    "           Note: 'buffer' can be a framebuf object.\n" \
    " Refer to the provided example for the usage.\n"

#define LCD_TBD_HELP_TEXT "Help text is not available yet.\n"

#define LCD_NEW_HELP_TEXT   LCD_TBD_HELP_TEXT
#define LCD_STR_HELP_TEXT   LCD_TBD_HELP_TEXT
#define LCD_CLEAR_HELP_TEXT LCD_TBD_HELP_TEXT
#define LCD_MODE_HELP_TEXT  LCD_TBD_HELP_TEXT
#define LCD_COLOR_HELP_TEXT LCD_TBD_HELP_TEXT
#define LCD_LINE_HELP_TEXT  LCD_TBD_HELP_TEXT
#define LCD_BLIT_HELP_TEXT  LCD_TBD_HELP_TEXT

#endif  // DEFINE_LCD_HELP_H