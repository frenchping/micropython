/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "py/runtime.h"

extern const mp_obj_module_t adc_group_type;

STATIC const mp_rom_map_elem_t sensor_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_sensor) },

    { MP_ROM_QSTR(MP_QSTR_ADC_Group),   MP_ROM_PTR(&adc_group_type) },
// #endif // ENABLE_CAMERA_MT9V032

};
STATIC MP_DEFINE_CONST_DICT(sensor_module_globals, sensor_module_globals_table);

const mp_obj_module_t mp_sensor_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sensor_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_sensor, mp_sensor_module);
