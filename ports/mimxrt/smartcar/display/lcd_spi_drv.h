/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Philipp Ebensberger
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

#include "py/runtime.h"
#include "lcd_fonts.h"

#ifndef LCD_SPI_DRV_INCLUDE_H
#define LCD_SPI_DRV_INCLUDE_H

// Define the full screen height length of the display
#define LCD350_WIDTH	480	//LCD width
#define LCD350_HEIGHT   320	//LCD height
#define LCD350_COLORS	16

#define LCD200_WIDTH	320	//LCD width
#define LCD200_HEIGHT   240	//LCD height
#define LCD200_COLORS	16

#define LCD154_WIDTH	240
#define LCD154_HEIGHT	240
#define LCD154_COLORS	16

#define LCD180_WIDTH	160
#define LCD180_HEIGHT	128
#define LCD180_COLORS	16

typedef enum {
    SPI_LCD_200 = 0,    // 2.0" ST7789, IPS 240x320
    SPI_LCD_180,        // 1.8" ST7735, FTF 128x160
	SPI_LCD_350,
	SPI_LCD_154,
	SPI_LCD_NUM_TYPE,
} LCD_Type;


typedef enum {
    ORIENT_0 = 0,   // Origin is on upper-left corner of screen.
                    // X: Left ==> Right, Y: Top ==> Bottom
    ORIENT_90,      // Origin is on upper-right corner of screen. Rotate anti-clock 90-degree
                    // X: Top ==> Bottom, Y: Right ==> Left
    ORIENT_180,     // Origin is on lower-right corner of screen. Rotate anti-clock 180-degree
                    // X: Right ==> Left, Y: Bottom ==> Top
    ORIENT_270,     // Origin is on lower-left corner of screen. Rotate anti-clock 270-degree
                    // X: Bottom ==> Top, Y: Left ==> Right
    // Bellow are mirror Y of above
    ORIENT_MIRROR_0,    // X: Left ==> Right, Y: Bottom ==> Top
    ORIENT_MIRROR_90,   // X: Top ==> Bottom, Y: Left ==> Right
    ORIENT_MIRROR_180,  // X: Right ==> Left, Y: Top ==> Bottom
    ORIENT_MIRROR_270,  // X: Bottom ==> Top, Y: Right ==> Left

    ORIENT_MAX
} Display_Orientation;
#define DEFAULT_ORIENTATION ORIENT_90

typedef struct _Rect_t {
    uint16_t    x0;
    uint16_t    y0;
    uint16_t    width;
    uint16_t    height;
} Rect_t;


typedef struct _lcd_user_drv_t {
    void* user_obj;
    int (*screen_init)(void *obj);
    int (*set_mode)(void *obj, int mode);
    int (*set_window)(void *obj, Rect_t *rect);
} lcd_user_drv_t;

typedef struct _lcd_display_t {
    LPSPI_Type *spi_base;       // The SPI bus

    lcd_user_drv_t user_ops;    // User operation functions

    const machine_pin_obj_t *rst_pin;
    const machine_pin_obj_t *dc_pin;
    const machine_pin_obj_t *cs_pin;
    const machine_pin_obj_t *sdo_pin;
    const machine_pin_obj_t *sck_pin;

    // Elements used for normal raster mode
    uint8_t     lpspi_id;
	uint8_t     lcd_type;
    uint8_t		Color_Depth;
    // Physical dimension
    uint16_t    Screen_Height;
    uint16_t    Screen_Width;
    // Viewing dimension. Depend on the display mode
    uint16_t    Height;
    uint16_t    Width;

	uint16_t 	Origin_shift_x;
	uint16_t	Origin_shift_y;
	Display_Orientation Orientation;

	uint16_t    Color_Foreground;
    uint16_t    Color_Background;
} lcd_display_t;


// This type contains basic attributes of a panel. 
typedef struct _lcd_hw_attribute_t {
	void (*lcd_init)	(lcd_display_t *lcd);
	void (*lcd_set_mode)(lcd_display_t *lcd, Display_Orientation mode);
    uint16_t    lcd_width;
    uint16_t    lcd_height;
    uint8_t     color_depth;
} lcd_hw_attribute_t;

void spi_lcd_write_cmd(lcd_display_t *lcd, const uint8_t *cmd, int len);
void spi_lcd_write_data(lcd_display_t *lcd, uint16_t Data);

void spi_lcd_screen_init(lcd_display_t *lcd);
void spi_lcd_set_window(lcd_display_t *lcd, Rect_t *rect);
void spi_lcd_set_mode(lcd_display_t *lcd, Display_Orientation mode);

void spi_lcd_fill_color(lcd_display_t *lcd, uint16_t color, uint32_t length);
void spi_lcd_fill_area(lcd_display_t *lcd, uint16_t *buf, uint32_t length);
void spi_lcd_draw_line (lcd_display_t *lcd, Rect_t *rect, uint16_t color, uint16_t thick);

void spi_lcd_dma_init(lcd_display_t *lcd, uint32_t dma_chn);
void spi_lcd_dma_fill(lcd_display_t *lcd, uint16_t *buf, uint32_t length, uint32_t dma_chn);
bool spi_lcd_dma_done();
#endif  // LCD_SPI_DRV_INCLUDE_H