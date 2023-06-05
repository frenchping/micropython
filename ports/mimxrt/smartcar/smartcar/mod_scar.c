/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "py/runtime.h"

extern const mp_obj_module_t camera_type;
extern const mp_obj_module_t adc_group_type;

STATIC const mp_rom_map_elem_t smartcar_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_smartcar) },
// #ifdef  ENABLE_CAMERA_MT9V032
    { MP_ROM_QSTR(MP_QSTR_MT9V032),     MP_ROM_PTR(&camera_type) },
// #endif // ENABLE_CAMERA_MT9V032
    { MP_ROM_QSTR(MP_QSTR_ADC_Group),   MP_ROM_PTR(&adc_group_type) },
};
STATIC MP_DEFINE_CONST_DICT(smartcar_module_globals, smartcar_module_globals_table);

const mp_obj_module_t mp_smartcar_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&smartcar_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_smartcar, mp_smartcar_module);
