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

#ifndef DEFINE_LCD_DRV_HELP_H
#define DEFINE_LCD_DRV_HELP_H

typedef enum _lcd_drv_help_id {
    LCD_DRV_NEW_HELP = 0,   // Constructor
    LCD_DRV_INIT_HELP,      // init()
    LCD_DRV_MODE_HELP,      // set_mode()
    LCD_DRV_AREA_HELP,      // ser_area()
    LCD_DRV_CMD_HELP,       // write_cmd()
    LCD_DRV_DATA_HELP,      // write_data()
    LCD_DRV_LAST_HELP
} lcd_drv_help_t;


#define LCD_DRV_HELP_TEXT \
    "LCD_Drv class is designed to drive the LCD panel interface. There is a\n" \
    "builtin support of a 320x240 color panel which uses a ST7789 driver chip.\n" \
    "User may derive this class and define own methods to drive a panel with a\n" \
    "different driver chip.\n" \
    "Functions (methods) in LCD class output the data to LCD_Drv class. Then\n" \
    "LCD_Drv renders data to the display panel.\n" \
    "-------------------------------\n" \
    "Following methods(functions) are in LCD_Drv:\n"\
    " Constructor:\n" \
    "   0. class display.LCD_Drv()\n" \
    " Methods:\n" \
    "   1. LCD_Drv.init()          initialize the panel.\n" \
    "   2. LCD_Drv.set_mode()      set panel orientation.\n" \
    "   3. LCD_Drv.set_area()      define the drawing window.\n" \
    "   4. LCD_Drv.write_cmd()     output a command byte to driver chip.\n" \
    "   5. LCD_Drv.write_data()    output a data byte to driver chip.\n" \
    " Constants:\n" \
    "   LCD200_TYPE    the panel type is builtin supported 2.0 inch\n"

#define LCD_DRV_NEW_HELP_TEXT \
    "class display.LCD_Drv(SPI_INDEX, DC_PIN, RST_PIN, *, ...)\n" \
    "   Create an object represents the driver.\n" \
    "    -- SPI_INDEX: the id of SPI interface. (Note, this is not hardware id)\n" \
    "    -- DC_PIN:    a machine.Pin object represents the DC pin.\n" \
    "    -- RST_PIN:   a machine.Pin object represents the Reset pin.\n" \
    "   Additional keyword-only parameters are as following:\n" \
    "    -- BAUDRATE:  define the SPI clock rate. Default is 6MHz.\n" \
    "    -- LCD_WIDTH: define width of the display panel. Default is 0.\n" \
    "    -- LCD_HEIGHT:define height of the display panel. Default is 0.\n" \
    "    -- LCD_TYPE:  define the type of the display panel. Default is -1.\n" \
    "           If LCD_TYPE = LCD_Drv.LCD200_TYPE, it means a builtin supported\n" \
    "           2.0 inch panel is used. In this case, LCD_WIDTH and/or LCD_HEIGHT\n" \
    "           can be ignored.\n" \
    "           If LCD_TYPE != LCD_Drv.LCD200_TYPE, it means an user self-defined\n" \
    "           driver is user. Therefore, either LCD_WIDTH or LCD_HEIGHT can not\n" \
    "           be 0. User need to override either init(), set_mode() or set_area()\n" \
    "           if certain operation(s) is difference than ST7789.\n"

#define LCD_DRV_INIT_HELP_TEXT \
    "LCD_Drv.init()\n" \
    "   This method does not have any parameter.\n" \
    "   It is called by LCD class when its constructor is called. It is used to\n" \
    "   output a series of commands to initialize the display panel.\n"\
    "   User has to derive the LCD_Drv class with own implementation of this method.\n" \
    " Refer to the provided example for the usage.\n"

#define LCD_DRV_MODE_HELP_TEXT \
    "LCD_Drv.set_mode(mode)\n" \
    "   Set the orientation mode of the display panel as 'mode'.\n" \
    "       mode = 0: rotate 0 degree\n" \
    "       mode = 1: rotate 90 degrees\n" \
    "       mode = 2: rotate 180 degrees\n" \
    "       mode = 3: rotate 270 degrees\n" \
    "       mode = 4: rotate 0 degree, mirrored\n" \
    "       mode = 5: rotate 90 degrees, mirrored\n" \
    "       mode = 6: rotate 180 degrees, mirrored\n" \
    "       mode = 7: rotate 270 degrees, mirrored\n" \
    "     Please refer to the driver chip document about the definition of display\n" \
    "     orientation.\n" \
    "   User has to derive the LCD_Drv class with own implementation of this method\n" \
    "   that issues necessary command(s) according to given parameter.\n" \
    " Refer to the provided example for the usage.\n"

#define LCD_DRV_AREA_HELP_TEXT \
    "LCD_Drv.set_area(x, y, w, h)\n" \
    "   Define a rectangle area which will be updated on display panel.\n" \
    " Refer to the provided example for the usage.\n"

#define LCD_DRV_CMD_HELP_TEXT \
    "LCD_Drv.write_cmd(cmds)\n" \
    "   Write a series of commands to the driver chip.\n" \
    "       Parameter 'cmds' is a tuple that contains a series of bytearray objects.\n" \
    "       The first byte of such bytearray is the command and followed by bytes of\n" \
    "       command parameters.\n" \
    " Refer to the provided example for the usage.\n"

#define LCD_DRV_DATA_HELP_TEXT  \
    "LCD_Drv.write_data(buffer)\n" \
    "   Output the pixels in 'buffer' to area defined by LCD_Drv.set_area().\n" \
    "     Parameter 'buffer' is an object with a buffer protocol.\n" \
    " Refer to the provided example for the usage.\n"

#endif  // DEFINE_LCD_DRV_HELP_H