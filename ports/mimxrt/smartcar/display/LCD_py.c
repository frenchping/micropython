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

#include "pin.h"

#include "fsl_iomuxc.h"
#include "fsl_lpspi.h"

#include "LCD_Help.h"
#include "LCD_py.h"
#include "lcd_bg_print.h"

int lcd_dma_chn = -1;

#if MICROPY_PY_FRAMEBUF
/*
    Definition here are inside modframebuf.c,
    the Github tree does not open it to other module.
*/
extern const mp_obj_module_t mp_module_framebuf;

typedef struct _mp_obj_framebuf_t {
    mp_obj_base_t base;
    mp_obj_t buf_obj; // need to store this to prevent GC from reclaiming buf
    void *buf;
    uint16_t width, height, stride;
    uint8_t format;
} mp_obj_framebuf_t;

/// @brief Given an object. Validate if it is a framebuf object
/// @return 
mp_obj_framebuf_t *get_framebuf_obj(mp_obj_t obj_in)
{
    // Check obj_in is object of class FrameBuffer
    mp_obj_dict_t *framebuf_dict = mp_module_framebuf.globals;
    mp_map_elem_t *elem = mp_map_lookup(&framebuf_dict->map, MP_ROM_QSTR(MP_QSTR_FrameBuffer), MP_MAP_LOOKUP);
    if (mp_obj_is_type(obj_in, (mp_obj_type_t*)elem->value)) {
        return MP_OBJ_TO_PTR(obj_in);
    }
    else
        mp_raise_TypeError(MP_ERROR_TEXT("last arg has to be object of FrameBuffer."));
}
#endif  // MICROPY_PY_FRAMEBUF


STATIC mp_obj_t lcd_help(size_t n_args, const mp_obj_t *args) {
    if (n_args == 0)
        mp_print_str(MP_PYTHON_PRINTER, LCD_HELP_TEXT);
    else {
        int help_id = mp_obj_get_int(args[0]);
        char *help_text[LCD_LAST_HELP] = {
            LCD_NEW_HELP_TEXT,
            LCD_STR_HELP_TEXT,
            LCD_CLEAR_HELP_TEXT,
            LCD_MODE_HELP_TEXT,
            LCD_COLOR_HELP_TEXT,
            LCD_LINE_HELP_TEXT,
            LCD_BLIT_HELP_TEXT
        };
        if (help_id >= 0 && help_id < LCD_LAST_HELP)
            mp_print_str(MP_PYTHON_PRINTER, help_text[help_id]);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_help_func, 0, 1, lcd_help);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(lcd_help_obj, MP_ROM_PTR(&lcd_help_func));

STATIC void lcd_obj_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    display_lcd_obj_t *self = self_in;
    lcd_display_t *lcd = &self->lcd_drv->lcd;
    mp_printf(print, "    ---- LCD Object ----\r\n");
    mp_printf(print, "\tCS Pin = %s\r\n", qstr_str(lcd->cs_pin->name));
    mp_printf(print, "\tDC Pin = %s\r\n", qstr_str(lcd->dc_pin->name));
    mp_printf(print, "\tRST Pin = %s\r\n", qstr_str(lcd->rst_pin->name));
    mp_printf(print, "\tWidth = %d, Height = %d\r\n", lcd->Width, lcd->Height);
    mp_printf(print, "\tLCD_Type = %d; Orientation mode = %d\r\n", lcd->lcd_type, lcd->Orientation);
}

/// \classmethod \constructor()
/// Create and return an SPI_LCD object.
///
/// LCD()
STATIC mp_obj_t lcd_obj_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    mp_obj_type_t *obj_in = (mp_obj_type_t*)MP_OBJ_TO_PTR(args[0]);
    const mp_obj_type_t *usr_drv_type = obj_in->base.type;

    if (!mp_obj_is_subclass_fast(usr_drv_type, MP_OBJ_FROM_PTR(&display_lcd_drv_type)))
        mp_raise_TypeError(MP_ERROR_TEXT("Provided object has to be LCD_Drv or derived from LCD_Drv."));

    lcd_drv_obj_t *lcd_drv = (lcd_drv_obj_t*)MP_OBJ_TO_PTR(args[0]);
    if (!mp_obj_is_type(obj_in, &display_lcd_drv_type)) {
        // To get the object of LCD_Drv's super class
        // Get the address and 'self' of the super(LCD_Drv, this).init
        // This 'self' is the object of super class
        mp_obj_t dest[3];
        dest[1] = (mp_obj_t)usr_drv_type;
        dest[2] = args[0];
        mp_load_super_method(MP_QSTR_init, dest);
        lcd_drv = dest[1];
    }

    if (usr_drv_type == &display_lcd_drv_type) {
        if (lcd_drv->lcd.lcd_type == 0xFF)
            check_override_func(obj_in, &lcd_drv->lcd);
    }
    else {
        // The driver object is a user-defined class
        // Check if several functions are overridden.
        check_override_func(obj_in, &lcd_drv->lcd);
    }
    
    display_lcd_obj_t *self = mp_obj_malloc(display_lcd_obj_t, &display_lcd_type);
    self->lcd_drv = lcd_drv;
    lcd_drv->lcd.Color_Foreground = 0xFFFF;   // White
    lcd_drv->lcd.Color_Background = 0;        // Black

    spi_lcd_screen_init(&lcd_drv->lcd);

#include "dma_manager.h"
    if (lcd_dma_chn == -1)  // Do not allocate more than one time
        lcd_dma_chn = allocate_dma_channel();
    dma_init();
    spi_lcd_dma_init(&lcd_drv->lcd, lcd_dma_chn);

    return MP_OBJ_FROM_PTR(self);
}

void lcd_str_screen(lcd_display_t *lcd, display_string_t *dstr, const Asc_Font_t *fontp)
{
    int font_buf_size = fontp->Width * fontp->Height * sizeof(uint16_t);
    uint16_t *BMP_buf = (uint16_t*)m_malloc(font_buf_size);
    int pos_x = dstr->loc_x;
    int pos_y = dstr->loc_y;
    char *string = (char*)dstr->str;

    if ((pos_y + fontp->Height) > lcd->Height)
        return;      // Exceed the screen height
    for (int ichr = 0; ichr < dstr->length; ichr++) {
        if (pos_x > lcd->Width)
            break;
        int chr = string[ichr] - ' ';
        uint16_t *bmp_buf = BMP_buf;
        const uint8_t *cfont = fontp->Fonts + chr * fontp->Bytes; // Pointer to the character font
        for (int fonty = 0; fonty < fontp->Height; fonty++) {
            uint16_t bmp = *cfont++;
            if (fontp->BytesOnWidth == 2)
                bmp |= *cfont++ << 8;
            uint16_t mask = 1;
            for (int fontx = 0; fontx < fontp->Width; fontx++) {
                if (bmp & mask)
                    *bmp_buf = dstr->color;
                else
                    *bmp_buf = lcd->Color_Background;
                bmp_buf++;
                mask <<= 1;
            }
        }
        Rect_t rect = { pos_x, pos_y, fontp->Width, fontp->Height };
        spi_lcd_set_window(lcd, &rect);
        spi_lcd_fill_area(lcd, BMP_buf, font_buf_size / 2);
        pos_x += fontp->Width;
        
    }
    m_free(BMP_buf);
}

void lcd_str_helper(size_t n_args, const mp_obj_t *args, const Asc_Font_t *fontp)
{
    display_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    lcd_display_t *lcd = &self->lcd_drv->lcd;

    uint16_t    pos_x = mp_obj_get_int(args[1]);
    uint16_t    pos_y = mp_obj_get_int(args[2]);
    mp_buffer_info_t strinfo;       // The text to show
    mp_get_buffer_raise(args[3], &strinfo, MP_BUFFER_READ);
    uint16_t    color = lcd->Color_Foreground;
    if (n_args > 4)
        color = mp_obj_get_int(args[4]);

    mp_obj_framebuf_t *framebuf = NULL;
    if (n_args > 5) {
        if (mp_const_none == MP_OBJ_TO_PTR(args[5])) {
            // If the last argv is exist but is None, this means show characters in background.
            bg_print_task_t *task = m_new(bg_print_task_t, 1);
            task->pos_x = pos_x;
            task->pos_y = pos_y;
            task->fg_color = color;
            task->bg_color = lcd->Color_Background;
            task->str_buf = strinfo.buf;
            task->str_len = strinfo.len;
            task->fontp = fontp;
            task->lcd = lcd;
            task->next = NULL;
            bg_print_sched(task);
            return;
        }
#if MICROPY_PY_FRAMEBUF
        else
            framebuf = get_framebuf_obj(args[5]);
#endif  // MICROPY_PY_FRAMEBUF
    }

    if (framebuf == NULL) {
        display_string_t dstr = { pos_x, pos_y, strinfo.buf, strinfo.len, color};
        lcd_str_screen(lcd, &dstr, fontp);
    }
#if MICROPY_PY_FRAMEBUF    
    else {
        // Draw the character to the given framebuf
        uint8_t *string = (uint8_t*)strinfo.buf;
        for (int ichr = 0; ichr < strinfo.len; ichr++) {
            int chr = string[ichr] - ' ';
            if ((pos_x + fontp->Width) > framebuf->width)
                break;
            if ((pos_y + fontp->Height) > framebuf->height)
                break;
            const uint8_t *cfont = fontp->Fonts + chr * fontp->Bytes; // Pointer to the character font
            uint16_t *bmp_buf = (uint16_t*)framebuf->buf + pos_y * framebuf->stride + pos_x;
            for (int fonty = 0; fonty < fontp->Height; fonty++) {
                uint16_t bmp = *cfont++;
                if (fontp->BytesOnWidth == 2)
                    bmp |= *cfont++ << 8;
                uint16_t mask = 1;
                for (int fontx = 0; fontx < fontp->Width; fontx++) {
                    if (bmp & mask)
                        *bmp_buf = color;
                    // Assume the font backgroud is transparent
                    bmp_buf++;
                    mask <<= 1;
                }
                bmp_buf += framebuf->stride - fontp->Width;
            }
            pos_x += fontp->Width;
        }
    }
#endif  // MICROPY_PY_FRAMEBUF    
}

// classmethon str16(x, y, str, color=0xFFFF, framebuf )
//  Show the given string on the given location
STATIC mp_obj_t lcd_str12(size_t n_args, const mp_obj_t *args)
{
    lcd_str_helper(n_args, args, &Asc_Font12);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_str12_obj, 4, 6, lcd_str12);

STATIC mp_obj_t lcd_str16(size_t n_args, const mp_obj_t *args)
{
    lcd_str_helper(n_args, args, &Asc_Font16);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_str16_obj, 4, 6, lcd_str16);

STATIC mp_obj_t lcd_str24(size_t n_args, const mp_obj_t *args)
{
    lcd_str_helper(n_args, args, &Asc_Font24);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_str24_obj, 4, 6, lcd_str24);

STATIC mp_obj_t lcd_str32(size_t n_args, const mp_obj_t *args)
{
    lcd_str_helper(n_args, args, &Asc_Font32);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_str32_obj, 4, 6, lcd_str32);

STATIC mp_obj_t lcd_clear(size_t n_args, const mp_obj_t *args)
{
    display_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    lcd_display_t *lcd = &self->lcd_drv->lcd;
    if (n_args > 1)
        lcd->Color_Background = mp_obj_get_int(args[1]);
    Rect_t rect = { 0, 0, lcd->Width, lcd->Height };
    spi_lcd_set_window(lcd, &rect);
    spi_lcd_fill_color(lcd, lcd->Color_Background, lcd->Width * lcd->Height);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_clear_obj, 1, 2, lcd_clear);

STATIC mp_obj_t lcd_mode(mp_obj_t self_in, mp_obj_t mode_in)
{
    display_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int mode = mp_obj_get_int(mode_in);
    spi_lcd_set_mode(&self->lcd_drv->lcd, mode);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(lcd_mode_obj, lcd_mode);

STATIC mp_obj_t lcd_color(mp_obj_t self_in, mp_obj_t fg_in, mp_obj_t bg_in)
{
    display_lcd_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int fg = mp_obj_get_int(fg_in);
    int bg = mp_obj_get_int(bg_in);
    self->lcd_drv->lcd.Color_Foreground = fg;
    self->lcd_drv->lcd.Color_Background = bg;
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_3(lcd_color_obj, lcd_color);

// classmethon line( x0, y0, x1, y1, color=fg, thickness=1)
// x0, y0 are starting point
// x1, y1 are ending point
// default color is the white
// thickness is default as 1
STATIC mp_obj_t lcd_line(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    display_lcd_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    enum { ARG_x0, ARG_y0, ARG_x1, ARG_y1, ARG_color, ARG_thick};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_x0, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_y0, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_x1, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_y1, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_color, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} },
        { MP_QSTR_thick, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 1} },
    };
    struct {        // parse args
        mp_arg_val_t x0, y0, x1, y1;
        mp_arg_val_t color, thick;
    } args;
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, (mp_arg_val_t*)&args);

    int color = args.color.u_int;
    if (color == -1)
        color = self->lcd_drv->lcd.Color_Foreground;
    Rect_t rect = {args.x0.u_int, args.y0.u_int, args.x1.u_int, args.y1.u_int};
    spi_lcd_draw_line(&self->lcd_drv->lcd, &rect, color, args.thick.u_int);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(lcd_line_obj, 5, lcd_line);

static void proto_blit(display_lcd_obj_t *lcd_obj, Rect_t *rect, mp_buffer_info_t *buf)
{
    spi_lcd_set_window(&lcd_obj->lcd_drv->lcd, rect);
    spi_lcd_fill_area(&lcd_obj->lcd_drv->lcd, buf->buf, buf->len / 2);
}

/// @brief Copy a buffer to the screen
/// \classmethod lcd_blit(x, y, width, height, framebuf)
STATIC mp_obj_t lcd_blit(size_t n_args, const mp_obj_t *args)
{
    display_lcd_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint16_t pos_x = mp_obj_get_int(args[1]);
    uint16_t pos_y = mp_obj_get_int(args[2]);
    uint16_t width = mp_obj_get_int(args[3]);
    uint16_t height= mp_obj_get_int(args[4]);
    mp_buffer_info_t buffer;
    mp_get_buffer_raise(args[5], &buffer, MP_BUFFER_READ);
    if (buffer.len < width * height * 2)
        mp_raise_ValueError(MP_ERROR_TEXT("Provided buf is too short."));

    Rect_t rect = { pos_x, pos_y, width, height };
    proto_blit(self, &rect, &buffer);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_blit_obj, 6, 6, lcd_blit);

// Output one character in background mode. Otherwise, no effect.
// Return True if these was something output.
// If 'waitDma' is True, do not return until the output is done.
STATIC mp_obj_t lcd_outc(size_t n_args, const mp_obj_t *args)
{  
    bool waitDma = false;
    if (n_args > 1)
        waitDma = mp_obj_is_true(args[1]);
    return mp_obj_new_bool(bg_print(waitDma));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_outc_obj, 1, 2, lcd_outc);

STATIC const mp_rom_map_elem_t lcd_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_help),    MP_ROM_PTR(&lcd_help_obj) },

    { MP_ROM_QSTR(MP_QSTR_str12),   MP_ROM_PTR(&lcd_str12_obj) },
    { MP_ROM_QSTR(MP_QSTR_str16),   MP_ROM_PTR(&lcd_str16_obj) },
    { MP_ROM_QSTR(MP_QSTR_str24),   MP_ROM_PTR(&lcd_str24_obj) },
    { MP_ROM_QSTR(MP_QSTR_str32),   MP_ROM_PTR(&lcd_str32_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),   MP_ROM_PTR(&lcd_clear_obj) },

    { MP_ROM_QSTR(MP_QSTR_mode),    MP_ROM_PTR(&lcd_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_color),   MP_ROM_PTR(&lcd_color_obj) },

    { MP_ROM_QSTR(MP_QSTR_line),    MP_ROM_PTR(&lcd_line_obj) },
    // { MP_ROM_QSTR(MP_QSTR_tile),    MP_ROM_PTR(&lcd_tile_obj) },
    // { MP_ROM_QSTR(MP_QSTR_load_bmp),MP_ROM_PTR(&lcd_load_bmp_obj) },
    { MP_ROM_QSTR(MP_QSTR_blit),    MP_ROM_PTR(&lcd_blit_obj) },

    { MP_ROM_QSTR(MP_QSTR_outchar), MP_ROM_PTR(&lcd_outc_obj) },
    // { MP_ROM_QSTR(MP_QSTR_font),     MP_ROM_PTR(&lcd_font_obj) },
    // { MP_ROM_QSTR(MP_QSTR_cursor),   MP_ROM_PTR(&lcd_cursor_obj) },
};
STATIC MP_DEFINE_CONST_DICT(lcd_locals_dict, lcd_locals_dict_table);

STATIC const display_lcd_p_t lcd_protocol_func = {
    .blit = proto_blit,
};

MP_DEFINE_CONST_OBJ_TYPE(
    display_lcd_type,
    MP_QSTR_LCD,
    MP_TYPE_FLAG_NONE,
    make_new, lcd_obj_make_new,
    print, lcd_obj_print,
    protocol, &lcd_protocol_func,
    locals_dict, &lcd_locals_dict
    );
