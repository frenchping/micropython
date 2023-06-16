/*
 * oled_py.c
 *
 *  Created on: Nov 15, 2021
 *      Author: nxa19017
 * 
 *  Micropython functions for manipulate 0.96 inch OLED.
 *  This module superseed functions in zfp_OLED.c
 * 
 */

#include "py/runtime.h"
#include "py/stream.h"
#include "fsl_common.h"
#include "extmod/machine_spi.h"
#include "extmod/virtpin.h"

extern const mp_obj_type_t smartcar_oled_type;

#define X_WIDTH 128
#define Y_WIDTH 64

typedef struct _smartcar_oled_obj_t {
    mp_obj_base_t base;
    mp_obj_base_t *spi;
    mp_obj_base_t *dc;
    mp_obj_base_t *res;
    uint8_t x, y;
    uint8_t fontsize;
} smartcar_oled_obj_t;

// STATIC smartcar_oled_obj_t oled_obj;

static const uint8_t init_CMD[] = {
		0xAE,		// Display off
		0x00, 0x10,	// Column address 0x00
		0x40,		// Set display RAM display start line register from
		0x81, 0xCF,	// Set Contrast Control
		0xA1,		// Set Segment Re-map, column address 127 is mapped to SEG0. Left/Right mirror
		0xC8,		// Set COM Output Scan Direction. Remapped mode. Scan from COM[N-1] to COM0
		0xA6,		// Set Normal Display
		0xA8, 0x3f,	// Set Multiplex Ratio
	    0xD3, 0x00,	// Set Display Offset
		0xd5, 0x80,	// Set Display Clock Divide Ratio/Oscillator Frequency
		0xD9, 0xF1,	// Set Pre-charge Period
		0xDA, 0x12,	// Set COM Pins Hardware Configuration
		0xDB, 0x40,	// Level
		0x20, 0x02,	// Set Memory Addressing Mode: Page Addressing Mode
		0x8D, 0x14,
		0xA4,		// Entire Display ON: Output follows RAM content
		0xA6,		// Set Normal/Inverse Display

        0xAF,       // turn on oled panel
};

void oled_fill_func(smartcar_oled_obj_t *self, uint8_t data)
{
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t*)self->spi->type->protocol;
    mp_pin_p_t *dc_ioctl = (mp_pin_p_t*)self->dc->type->protocol;
    uint8_t cmds[3] = { 0xb0, 0x01, 0x10 };
    uint8_t buff[X_WIDTH];
    memset(buff, data, sizeof(buff));
    for (int y=0; y < 8; y++) {
        cmds[0] = 0xb0 + y;
        dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 0, NULL);
        spi_p->transfer(self->spi, sizeof(cmds), cmds, NULL);
        // mp_hal_delay_ms(1);

        dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 1, NULL);
        spi_p->transfer(self->spi, sizeof(buff), buff, NULL);
        // mp_hal_delay_ms(1);
    }
    self->x = self->y = 0;
}

mp_obj_t oled_set_pixel(const mp_obj_t *args, uint8_t dat)
{
    smartcar_oled_obj_t *self = (smartcar_oled_obj_t*)MP_OBJ_TO_PTR(args[0]);
    uint8_t x = mp_obj_int_get_uint_checked(args[1]);
    uint8_t y = mp_obj_int_get_uint_checked(args[2]);
    if (dat)
        dat = mp_obj_int_get_uint_checked(args[3]);
    if (x >= X_WIDTH || y >= Y_WIDTH)
        mp_raise_msg(&mp_type_ValueError, "wrong coordination");

    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t*)self->spi->type->protocol;
    mp_pin_p_t *dc_ioctl = (mp_pin_p_t*)self->dc->type->protocol;
    self->x = x;
    self->y = y;

    uint8_t cmds[3];
    cmds[0] = 0xb0+y;
    cmds[1] = ((x & 0xf0) >> 4) | 0x10;
    cmds[2] = (x & 0x0f) | 0x00;
    dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 0, NULL);
    spi_p->transfer(self->spi, sizeof(cmds), cmds, NULL);
    dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 1, NULL);
    spi_p->transfer(self->spi, sizeof(uint8_t), &dat, NULL);
    return mp_const_none;
}

STATIC mp_uint_t oled_stream_str(smartcar_oled_obj_t *self, const uint8_t *str, mp_uint_t size, uint8_t fontsize)
{
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t*)self->spi->type->protocol;
    mp_pin_p_t *dc_ioctl = (mp_pin_p_t*)self->dc->type->protocol;
    uint8_t x = self->x;
    uint8_t y = self->y;
    int ich;
    for (ich=0; ich < size; ich++) {
        uint8_t cmds[3];
        if (x >= (X_WIDTH - fontsize)) {
            x = 0;
            y++;
        }
        if (y >= Y_WIDTH)
            break;
        uint32_t ch = str[ich];
        if (ch < 0x20) {
            switch (ch) {
                case '\r':
                    x = 0;  break;
                case '\n':
                    y++;    break;
                default:
                    break;
            }
            continue;
        }
        ch -= 0x20;

        cmds[0] = 0xb0+y;
        cmds[1] = ((x & 0xf0) >> 4) | 0x10;
        cmds[2] = (x & 0x0f) | 0x00;
        dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 0, NULL);
        spi_p->transfer(self->spi, sizeof(cmds), cmds, NULL);

        dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 1, NULL);
        if (fontsize == 6) {
            extern const uint8_t oled_6x8[][6];
            spi_p->transfer(self->spi, 6, oled_6x8[ch], NULL);
            x += 6;
        }
        else if (fontsize==8) {
            extern const uint8_t oled_8x16[];
            ch *= 16;
            spi_p->transfer(self->spi, 8, &oled_8x16[ch], NULL);

            cmds[0]++;
            dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 0, NULL);
            spi_p->transfer(self->spi, sizeof(cmds), cmds, NULL);
            dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 1, NULL);
            spi_p->transfer(self->spi, 8, &oled_8x16[ch+8], NULL);
            x += 8;
        }
    }
    self->x = x;
    self->y = y;
    return ich;
}

/// \classmethod \constructor()
/// Create and return an ADC object.
///
/// Construct an ADC object with given ADCn and Ch.
///
STATIC mp_obj_t oled_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    extern const mp_obj_type_t machine_hard_spi_type;
    extern const mp_obj_type_t pyb_pin_type;

    mp_arg_check_num(n_args, n_kw, 3, 3, false);
    mp_obj_base_t *spi = (mp_obj_base_t*)MP_OBJ_TO_PTR(args[0]);
    if (!mp_obj_is_type(spi, &machine_hard_spi_type) && !mp_obj_is_type(spi, &mp_machine_soft_spi_type))
        mp_raise_msg(&mp_type_TypeError, MP_ERROR_TEXT("param1 has to be a SPI obj"));
    mp_obj_base_t *dc = (mp_obj_base_t*)MP_OBJ_TO_PTR(args[1]);
    if (!mp_obj_is_type(dc, &pyb_pin_type))
        mp_raise_msg(&mp_type_TypeError, MP_ERROR_TEXT("param2 (DC) has to be a Pin obj"));
    mp_obj_base_t *res = (mp_obj_base_t*)MP_OBJ_TO_PTR(args[2]);
    if (!mp_obj_is_type(res, &pyb_pin_type))
        mp_raise_msg(&mp_type_TypeError, MP_ERROR_TEXT("param3 (RES) has to be a Pin obj"));

    smartcar_oled_obj_t *self = m_new_obj(smartcar_oled_obj_t);
    memset(self, 0, sizeof(smartcar_oled_obj_t));
    self->base.type = &smartcar_oled_type;
    self->dc  = dc;
    self->res = res;
    self->spi = spi;
    self->x = self->y = 0;
    self->fontsize = 6;     // Default is smaller one.

    mp_pin_p_t *res_ioctl = (mp_pin_p_t*)res->type->protocol;
    // Reset the OLED
    res_ioctl->ioctl(res, MP_PIN_WRITE, 0, NULL);
    mp_hal_delay_ms(50);
    res_ioctl->ioctl(res, MP_PIN_WRITE, 1, NULL);
    mp_hal_delay_ms(1);

    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t*)spi->type->protocol;
    mp_pin_p_t *dc_ioctl = (mp_pin_p_t*)self->dc->type->protocol;
    dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 0, NULL);
    spi_p->transfer(spi, sizeof(init_CMD), init_CMD, NULL);     // Send initialize array

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t oled_fill(mp_obj_t self_in, mp_obj_t data_in)
{
    smartcar_oled_obj_t *self = self_in;
    uint8_t data = mp_obj_get_int(data_in);
    oled_fill_func(self, data);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(oled_fill_obj, oled_fill);

STATIC mp_obj_t oled_putpixel(size_t n_args, const mp_obj_t *args)
{
    // smartcar_oled_obj_t *self = args[0];
    return oled_set_pixel(args, 0xFF);
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(oled_putpixel_obj, 4, 4, oled_putpixel);

STATIC mp_obj_t oled_clrpixel(size_t n_args, const mp_obj_t *args)
{
    // smartcar_oled_obj_t *self = args[0];
    return oled_set_pixel(args, 0);
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(oled_clrpixel_obj, 3, 3, oled_clrpixel);

STATIC mp_obj_t oled_string(size_t n_args, const mp_obj_t *args)
{
    // smartcar_oled_obj_t *self = args[0];
    smartcar_oled_obj_t *self = (smartcar_oled_obj_t*)MP_OBJ_TO_PTR(args[0]);
    uint8_t x = mp_obj_int_get_uint_checked(args[1]);
    uint8_t y = mp_obj_int_get_uint_checked(args[2]);
    if (x >= X_WIDTH || y >= Y_WIDTH)
        mp_raise_msg(&mp_type_ValueError, "wrong coordination");
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[3], &bufinfo, MP_BUFFER_READ);

    self->x = x;
    self->y = y;
    oled_stream_str(self, (uint8_t*)bufinfo.buf, bufinfo.len, self->fontsize);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(oled_string_obj, 4, 4, oled_string);

STATIC mp_obj_t oled_fontsize(mp_obj_t self_in, mp_obj_t size_in)
{
    smartcar_oled_obj_t *self = self_in;
    uint8_t size = mp_obj_get_int(size_in);
    if (size != 6 && size != 8)
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("fontsize has to be 6 or 8"));
    self->fontsize = size;
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(oled_fontsize_obj, oled_fontsize);

STATIC mp_uint_t oled_stream_write(mp_obj_t o_in, const void *buf, mp_uint_t size, int *errcode) {
    mp_obj_t *o = MP_OBJ_TO_PTR(o_in);
    if (!mp_obj_is_type(o, &smartcar_oled_type)) {
        *errcode = ENODEV;
        return MP_STREAM_ERROR;
    }
    return oled_stream_str((smartcar_oled_obj_t*)o, (uint8_t*)buf, size, ((smartcar_oled_obj_t*)o)->fontsize);
}

// chinese(x, y, matrix, fontsize=16)
STATIC mp_obj_t oled_chinese(size_t n_args, const mp_obj_t *args)
{
    smartcar_oled_obj_t *self = (smartcar_oled_obj_t*)MP_OBJ_TO_PTR(args[0]);
    uint8_t x = mp_obj_int_get_uint_checked(args[1]);
    uint8_t y = mp_obj_int_get_uint_checked(args[2]);
    if (x >= X_WIDTH || y >= Y_WIDTH)
        mp_raise_msg(&mp_type_ValueError, "wrong coordination");
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[3], &bufinfo, MP_BUFFER_READ);
    uint fontsize = 16;
    if (n_args == 5)
        fontsize = mp_obj_int_get_uint_checked(args[4]);
    
    uint8_t *buf = bufinfo.buf;
    uint8_t cmds[3];
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t*)self->spi->type->protocol;
    mp_pin_p_t *dc_ioctl = (mp_pin_p_t*)self->dc->type->protocol;
    int word_bytes = fontsize * fontsize / 8;
    int word_num = bufinfo.len / word_bytes;
    for (int iwd = 0; iwd < word_num; iwd++) {
        cmds[0] = 0xb0+y;
        cmds[1] = ((x & 0xf0) >> 4) | 0x10;
        cmds[2] = (x & 0x0f) | 0x00;
        dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 0, NULL);
        spi_p->transfer(self->spi, sizeof(cmds), cmds, NULL);
        dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 1, NULL);
        spi_p->transfer(self->spi, fontsize, buf, NULL);

        cmds[0]++;
        buf += fontsize;
        dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 0, NULL);
        spi_p->transfer(self->spi, sizeof(cmds), cmds, NULL);
        dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 1, NULL);
        spi_p->transfer(self->spi, fontsize, buf, NULL);

        x += fontsize;
        buf += fontsize;
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(oled_chinese_obj, 4, 4, oled_chinese);

// image2(height, width, matrix, threshold)
STATIC mp_obj_t oled_image2(size_t n_args, const mp_obj_t *args)
{
    smartcar_oled_obj_t *self = (smartcar_oled_obj_t*)MP_OBJ_TO_PTR(args[0]);
    int height = mp_obj_get_int(args[1]);
    int slice = height / 8;
    if (height % 8)
        slice++;
    int width = mp_obj_get_int(args[2]);

    mp_buffer_info_t source;
    mp_get_buffer_raise(args[3], &source, MP_BUFFER_READ);
    int total = width * height;
    if (total > source.len)
        total = source.len;

    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t*)self->spi->type->protocol;
    mp_pin_p_t *dc_ioctl = (mp_pin_p_t*)self->dc->type->protocol;

    uint threshold = mp_obj_int_get_uint_checked(args[4]);
    uint8_t *buffer = m_new(uint8_t, width);
    uint8_t *pixel = source.buf;
    for (int row = 0; row < slice; row++) {
        memset(buffer, 0, width);
        for (int col = 0; total && (col < width); col++) {
            for (int line = 0; line < 8; line++) {
                if (*pixel > threshold)
                    buffer[col] |= 1 << line;
                pixel++;
                total--;
            }
        }
        uint8_t cmds[3] = { 0xb0+row, 0x10, 0x0 };
        dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 0, NULL);
        spi_p->transfer(self->spi, sizeof(cmds), cmds, NULL);
        dc_ioctl->ioctl(self->dc, MP_PIN_WRITE, 1, NULL);
        spi_p->transfer(self->spi, width, buffer, NULL);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(oled_image2_obj, 5, 5, oled_image2);

STATIC const mp_rom_map_elem_t oled_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_fill),    MP_ROM_PTR(&oled_fill_obj) },
    { MP_ROM_QSTR(MP_QSTR_putpixel),MP_ROM_PTR(&oled_putpixel_obj) },
    { MP_ROM_QSTR(MP_QSTR_clrpixel),MP_ROM_PTR(&oled_clrpixel_obj) },
    { MP_ROM_QSTR(MP_QSTR_string),  MP_ROM_PTR(&oled_string_obj) },
    { MP_ROM_QSTR(MP_QSTR_fontsize),MP_ROM_PTR(&oled_fontsize_obj) },
    { MP_ROM_QSTR(MP_QSTR_chinese), MP_ROM_PTR(&oled_chinese_obj) },

    { MP_ROM_QSTR(MP_QSTR_image2),  MP_ROM_PTR(&oled_image2_obj) },
};

STATIC MP_DEFINE_CONST_DICT(oled_locals_dict, oled_locals_dict_table);

STATIC const mp_stream_p_t oled_stream_p = {
    .write = oled_stream_write,
    .is_text = true,
};

const mp_obj_type_t smartcar_oled_type = {
    { &mp_type_type },
    .name = MP_QSTR_oled,
    .make_new = oled_make_new,
    .protocol = &oled_stream_p,
    .locals_dict = (mp_obj_dict_t *)&oled_locals_dict,
};
