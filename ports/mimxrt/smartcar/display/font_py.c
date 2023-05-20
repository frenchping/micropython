
#include "py/obj.h"
#include "py/runtime.h"

#include "lcd_fonts.h"

#include "oled_font.c"

#define FONT_HELP_TEXT  \
    "All functions in this module return a bytearray object. It contains\n"\
    "dot matrix data that corresponds to the first character of given str.\n"\
    "                       width height    format as\n"   \
    "  -- get_oled_6x8():     6      8    framebuf.MONO_VLSB\n" \
    "  -- get_oled_8x16():    8     16    framebuf.MONO_VLSB\n" \
    "  -- get_lcd_6x12():     6     12    framebuf.MONO_HMSB\n" \
    "  -- get_lcd_8x16():     8     16    framebuf.MONO_HMSB\n" \
    "  -- get_lcd_12x24():   12     24    framebuf.MONO_HMSB\n" \
    "  -- get_lcd_16x32():   16     32    framebuf.MONO_HMSB\n"

int get_code(mp_obj_t code_in)
{
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(code_in, &bufinfo, MP_BUFFER_READ);
    int code = *(uint8_t*)bufinfo.buf;
    if (code < ' ' && code >= '~')
        mp_raise_ValueError(MP_ERROR_TEXT("'code' has to be an ascii code"));
    return code - ' ';
}

static mp_obj_t get_font(const Asc_Font_t *font, mp_obj_t code_in)
{
    return mp_obj_new_bytearray_by_ref(font->Bytes, (void*)(font->Fonts + get_code(code_in) * font->Bytes));
}

STATIC mp_obj_t font_help() {
    mp_print_str(MP_PYTHON_PRINTER, FONT_HELP_TEXT);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(font_help_obj, font_help);

STATIC mp_obj_t get_oled_6x8(mp_obj_t code_in) {
    return mp_obj_new_bytearray_by_ref(6, (void*)oled_6x8[get_code(code_in)]);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(get_oled_6x8_obj, get_oled_6x8);

STATIC mp_obj_t get_oled_8x16(mp_obj_t code_in) {
    return mp_obj_new_bytearray_by_ref(16, (void*)(oled_8x16 + get_code(code_in)*16));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(get_oled_8x16_obj, get_oled_8x16);

STATIC mp_obj_t get_lcd_6x12(mp_obj_t code_in) {
    return get_font(&Asc_Font12, code_in);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(get_lcd_6x12_obj, get_lcd_6x12);

STATIC mp_obj_t get_lcd_8x16(mp_obj_t code_in) {
    return get_font(&Asc_Font16, code_in);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(get_lcd_8x16_obj, get_lcd_8x16);

STATIC mp_obj_t get_lcd_12x24(mp_obj_t code_in) {
    return get_font(&Asc_Font24, code_in);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(get_lcd_12x24_obj, get_lcd_12x24);

STATIC mp_obj_t get_lcd_16x32(mp_obj_t code_in) {
    return get_font(&Asc_Font32, code_in);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(get_lcd_16x32_obj, get_lcd_16x32);

STATIC const mp_rom_map_elem_t font_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_font) },
    { MP_ROM_QSTR(MP_QSTR_help),        MP_ROM_PTR(&font_help_obj) },
    { MP_ROM_QSTR(MP_QSTR_oled_6x8),    MP_ROM_PTR(&get_oled_6x8_obj) },
    { MP_ROM_QSTR(MP_QSTR_oled_8x16),   MP_ROM_PTR(&get_oled_8x16_obj) },
    { MP_ROM_QSTR(MP_QSTR_lcd_6x12),    MP_ROM_PTR(&get_lcd_6x12_obj) },
    { MP_ROM_QSTR(MP_QSTR_lcd_8x16),    MP_ROM_PTR(&get_lcd_8x16_obj) },
    { MP_ROM_QSTR(MP_QSTR_lcd_12x24),   MP_ROM_PTR(&get_lcd_12x24_obj) },
    { MP_ROM_QSTR(MP_QSTR_lcd_16x32),   MP_ROM_PTR(&get_lcd_16x32_obj) },
};

STATIC MP_DEFINE_CONST_DICT (mp_module_font_globals, font_globals_table );

const mp_obj_module_t mp_module_font = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_font_globals,
};

// MP_REGISTER_MODULE(MP_QSTR_font, mp_module_font);
