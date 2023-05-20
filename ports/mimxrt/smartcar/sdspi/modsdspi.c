/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "py/runtime.h"

extern const mp_obj_type_t sd_spi_type;

STATIC const mp_rom_map_elem_t sdspi_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_sdspi) },

    { MP_ROM_QSTR(MP_QSTR_SD_SPI),      MP_ROM_PTR(&sd_spi_type) },
// #endif // ENABLE_CAMERA_MT9V032

};
STATIC MP_DEFINE_CONST_DICT(sdspi_module_globals, sdspi_module_globals_table);

const mp_obj_module_t mp_sdspi_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sdspi_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_SD_SPI, mp_sdspi_module);
