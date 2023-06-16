/*
 * SPI_LCD_py.c
 *
 *  Created on: Dec 13, 2021
 *      Author: nxa19017
 * 
 *  Micropython functions for manipulate LCD panel
 *  This module superseed functions in SEEKFREE_TFT18.c
 * 
 */

#include "py/runtime.h"
#include "py/stream.h"
#include "py/smallint.h"

#include <rtthread.h>

#ifdef  BSP_USING_SPI_LCD
#include "drv_spi_lcd_funcs.h"
#define LCD_DEVICE_NAME "spi_lcd"
#else
#define LCD_DEVICE_NAME "lcd_18"
#endif // BSP_USING_SPI_LCD

#include "lcd_drv_op.h"
#include "SPI_LCD_py.h"

extern const mp_obj_type_t smartcar_spi_lcd_type;

typedef union _color_values {
    uint16_t color;
    struct _rgb_ {
        uint16_t b: 5;
        uint16_t g: 6;
        uint16_t r: 5;
    } RGB;
} color_value_t;

typedef struct _smartcar_lcd_obj_t {
    mp_obj_base_t base;
    rt_device_t     lcd_dev;
} smartcar_lcd_obj_t;

STATIC void lcd_self_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    smartcar_lcd_obj_t *self = self_in;
    color_value_t   fg, bg;
    display_info_t  info;
    rt_device_control(self->lcd_dev, LCD_GET_INFO, &info);
    fg.color = info.fg;
    bg.color = info.bg;
    Display_Info_t  sizes;
    rt_device_control(self->lcd_dev, SPI_LCD_GET_INFO, &sizes);

    mp_printf(print, "LCD:\tWidth = %d,\tHeight = %d,\tColor Depth = %d\r\n",
                            sizes.Width, sizes.Height, sizes.Color_Depth);
    mp_printf(print, "\tfont = %d,\tmode = %d,\tcursor = (%d, %d)\r\n", info.font, info.mode, info.x, info.y);
    mp_printf(print, "\tcolor = RGB(%d, %d, %d);\tbackground = RGB(%d, %d, %d)\r\n",
                            fg.RGB.r << 3, fg.RGB.g << 2, fg.RGB.b << 3, bg.RGB.r << 3, bg.RGB.g << 2, bg.RGB.b << 3);
}

/// \classmethod \constructor()
/// Create and return an SPI_LCD object.
///
/// LCD()
STATIC mp_obj_t lcd_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);

    smartcar_lcd_obj_t *self = m_new_obj(smartcar_lcd_obj_t);
    memset(self, 0, sizeof(smartcar_lcd_obj_t));
    self->base.type = &smartcar_spi_lcd_type;

    self->lcd_dev = rt_device_find(LCD_DEVICE_NAME);
    rt_device_open(self->lcd_dev, RT_DEVICE_OFLAG_WRONLY);

    return MP_OBJ_FROM_PTR(self);
}

// classmethon string(str)
//  Show the given string on the terminal at current corrdination
STATIC mp_obj_t lcd_string(mp_obj_t self_in, mp_obj_t str_in)
{
    smartcar_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(str_in, &bufinfo, MP_BUFFER_READ);

    if (rt_device_write(self->lcd_dev, -1, bufinfo.buf, bufinfo.len))
        mp_raise_OSError(RT_EBUSY);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(lcd_string_obj, lcd_string);

// classmethon print(x, y, str)
//  x,y are the absolute corrdination
//  show the given 'str' on the screen
// This is an asynchronous operation. It outputs the characters when the mpy engine is idle
STATIC mp_obj_t lcd_print(size_t n_args, const mp_obj_t *args)
{
    smartcar_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    cursor_position_t   pos;
    pos.x = mp_obj_get_int(args[1]);
    pos.y = mp_obj_get_int(args[2]);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[3], &bufinfo, MP_BUFFER_READ);

    rt_device_write(self->lcd_dev, *((uint32_t*)&pos), bufinfo.buf, bufinfo.len);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_print_obj, 4, 4, lcd_print);

STATIC mp_obj_t lcd_flush(mp_obj_t self_in)
{
    smartcar_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    rt_device_control(self->lcd_dev, SPI_LCD_PRINT_FLUSH, NULL);    // Will not return until finish
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(lcd_flush_obj, lcd_flush);

STATIC mp_obj_t lcd_cursor(size_t n_args, const mp_obj_t *args)
{
    smartcar_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    cursor_position_t   pos;
    if (n_args == 3) {
        pos.x = mp_obj_get_int(args[1]);
        pos.y = mp_obj_get_int(args[2]);
        rt_device_control(self->lcd_dev, LCD_SET_CURSOR, &pos);
        return mp_const_none;
    }
    else {
        rt_device_control(self->lcd_dev, LCD_GET_CURSOR, &pos);
        mp_obj_t tuple[2] = {
            MP_OBJ_NEW_SMALL_INT(pos.x),
            MP_OBJ_NEW_SMALL_INT(pos.y)
        };
        return MP_OBJ_FROM_PTR(mp_obj_new_tuple(2, tuple));
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_cursor_obj, 1, 3, lcd_cursor);

STATIC mp_obj_t lcd_clear(mp_obj_t self_in)
{
    smartcar_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    rt_device_control(self->lcd_dev, LCD_CLEAR, NULL);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(lcd_clear_obj, lcd_clear);

STATIC mp_obj_t lcd_clear_line(size_t n_args, const mp_obj_t *args)
{
    smartcar_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    cursor_position_t   pos = {
        .x  = 1,        // Number of lines to be cleared
        .y  = mp_obj_get_int(args[1]),  // Starting line to be cleared
    };
    if (n_args == 3)
        pos.x = mp_obj_get_int(args[2]);
    rt_device_control(self->lcd_dev, LCD_CLEAR_LINE, &pos);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_clear_line_obj, 2, 3, lcd_clear_line);

STATIC mp_obj_t lcd_mode(mp_obj_t self_in, mp_obj_t mode_in)
{
    smartcar_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int mode = mp_obj_get_int(mode_in);
    rt_device_control(self->lcd_dev, LCD_SET_MODE, &mode);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(lcd_mode_obj, lcd_mode);

STATIC mp_obj_t lcd_color(mp_obj_t self_in, mp_obj_t fg_in, mp_obj_t bg_in)
{
    smartcar_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint32_t fg = mp_obj_get_int(fg_in);
    uint32_t bg = mp_obj_get_int(bg_in);
    uint32_t color = (fg & 0xFFFF) | ((bg & 0xFFFF) << 16);
    rt_device_control(self->lcd_dev, LCD_SET_COLOR, &color);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(lcd_color_obj, lcd_color);

STATIC mp_obj_t lcd_font(mp_obj_t self_in, mp_obj_t font_in)
{
    smartcar_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int font = mp_obj_get_int(font_in);
    if (font < 0x20) {
        rt_device_control(self->lcd_dev, LCD_SET_FONT, &font);
    }
    else {
        rt_device_control(self->lcd_dev, LCD_GET_FONT, &font);
        return mp_obj_new_bytearray_by_ref(64, (void*)font);    // 64 is the largest font
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(lcd_font_obj, lcd_font);

// classmethon tile(x, y, pixel-array)
// x, y are display location
// pixel is a 'H' type array. First 2 elements are width & height.
// The array must have width*height+2 in length
STATIC mp_obj_t lcd_tile(size_t n_args, const mp_obj_t *args)
{
    smartcar_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint16_t pos_x = mp_obj_get_int(args[1]);
    uint16_t pos_y = mp_obj_get_int(args[2]);
    mp_buffer_info_t    pixels;
    mp_get_buffer_raise(args[3], &pixels, MP_BUFFER_READ);
    if (pixels.typecode != 'H' && pixels.typecode != 'I')
        mp_raise_TypeError("array has to be in type 'H' or 'I'");
    uint16_t *buffer = pixels.buf;
    int width = buffer[0];
    int height =buffer[1];
    if (width * height < (pixels.len/sizeof(uint16_t) - 2))
        mp_raise_ValueError("pixel array size is too short.");
#ifdef  BSP_USING_SPI_LCD
    {
        Display_ReqBlock_t req;
        req.rect.x0 = pos_x;
        req.rect.y0 = pos_y;
        req.rect.width = width;
        req.rect.height = height;
        req.buffer = (uint8_t*)(buffer + 2);
        req.buf_len = (pixels.len - 2 * sizeof(uint16_t)) / 2;
        rt_device_control(self->lcd_dev, SPI_LCD_DRAW_TILE, &req);
    }
#else
    uint32_t location = pos_x | (pos_y << 16);
    rt_device_write(self->lcd_dev, location, buffer, pixels.len);
#endif
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_tile_obj, 4, 4, lcd_tile);


// classmethon load_bmp(filename, top_shift=MP_SMALL_INT_MAX, left_shift=MP_SMALL_INT_MAX)
// Show the center portion of picture 'filename'.
// If top_shift and/or left_shift is given, the showing portion will start from there
STATIC mp_obj_t lcd_load_bmp(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_file,        MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_top_shift,   MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_left_shift,  MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
    };

    struct {    // parse args
        mp_arg_val_t file_name, top_shift, left_shift;
    } args;
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, (mp_arg_val_t*)&args);

    smartcar_lcd_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args.file_name.u_obj, &bufinfo, MP_BUFFER_READ);
    int bmpf = open(bufinfo.buf, O_RDONLY);
    if (bmpf == -1)
        mp_raise_ValueError("Can not open picture file.");

    BITMAPFILEHEADER	*fileh = m_new_obj(BITMAPFILEHEADER);
    BITMAPINFOHEADER    *infoh = m_new_obj(BITMAPINFOHEADER);
    read(bmpf, fileh, sizeof(BITMAPFILEHEADER));
    if (fileh->bfType != 0x4d42)     // 'BM'
        mp_raise_ValueError("Given file is not a BMP file.");
    read(bmpf, infoh, sizeof(BITMAPINFOHEADER));
    if (infoh->biBitCount != 24)
        mp_raise_ValueError("Given file is not a BMP 24-bit file.");
    m_del_obj(BITMAPFILEHEADER, fileh);
    m_del_obj(BITMAPINFOHEADER, infoh);

    show_bmp_24_t bmp24;
    bmp24.fd = bmpf;
    bmp24.top_shift = args.top_shift.u_int;
    bmp24.left_shift = args.left_shift.u_int;
    rt_device_control(self->lcd_dev, LCD_FILE_BMP24, &bmp24);

    close(bmpf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(lcd_load_bmp_obj, 2, lcd_load_bmp);

STATIC mp_obj_t smartcar_wfi()
{
    MICROPY_EVENT_POLL_HOOK
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(smartcar_wfi_static_obj, smartcar_wfi);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(smartcar_wfi_obj, MP_ROM_PTR(&smartcar_wfi_static_obj));

STATIC const mp_rom_map_elem_t lcd_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_string),  MP_ROM_PTR(&lcd_string_obj) },
    { MP_ROM_QSTR(MP_QSTR_print),   MP_ROM_PTR(&lcd_print_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush),   MP_ROM_PTR(&lcd_flush_obj) },
    { MP_ROM_QSTR(MP_QSTR_cursor),  MP_ROM_PTR(&lcd_cursor_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),   MP_ROM_PTR(&lcd_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_line),MP_ROM_PTR(&lcd_clear_line_obj) },

    { MP_ROM_QSTR(MP_QSTR_mode),    MP_ROM_PTR(&lcd_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_color),   MP_ROM_PTR(&lcd_color_obj) },
    { MP_ROM_QSTR(MP_QSTR_font),    MP_ROM_PTR(&lcd_font_obj) },

    { MP_ROM_QSTR(MP_QSTR_tile),    MP_ROM_PTR(&lcd_tile_obj) },
    { MP_ROM_QSTR(MP_QSTR_load_bmp),MP_ROM_PTR(&lcd_load_bmp_obj) },

    { MP_ROM_QSTR(MP_QSTR_wfi),     MP_ROM_PTR(&smartcar_wfi_obj) },

};
STATIC MP_DEFINE_CONST_DICT(lcd_locals_dict, lcd_locals_dict_table);

STATIC mp_uint_t smartcar_lcd_write(mp_obj_t self_in, const void *buf_in, mp_uint_t size, int *errcode) {
    smartcar_lcd_obj_t *self = self_in;
    const byte *buf = buf_in;
    *errcode = 0;
    return rt_device_write(self->lcd_dev, -1, buf, size);
}

STATIC mp_uint_t smartcar_lcd_ioctl(mp_obj_t obj, mp_uint_t request, uintptr_t arg, int *errcode)
{
    smartcar_lcd_obj_t *self = obj;
    *errcode = 0;
    switch (request) {
        case LCD_BMP_8BIT_QUAD:
        case LCD_BMP_8BIT: {
            show_picture_t *param = (show_picture_t*)arg;
            rt_device_control(self->lcd_dev, request, param);
            break;
        }
        default:
            break;
    }
    return 0;
}

STATIC const mp_stream_p_t lcd_stream_p = {
    // .read = smartcar_lcd_read,
    .write = smartcar_lcd_write,
    .ioctl = smartcar_lcd_ioctl,
    .is_text = true,
};

const mp_obj_type_t smartcar_spi_lcd_type = {
    { &mp_type_type },
    .name = MP_QSTR_LCD,
    .make_new = lcd_make_new,
    .protocol = &lcd_stream_p,
    .print = lcd_self_print,
    .locals_dict = (mp_obj_dict_t *)&lcd_locals_dict,
};
