/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* This version does not allow pins from 2 ADCs in our group */

#include "py/obj.h"
#include "py/runtime.h"
#include "py/objarray.h"
#include "pin.h"

#include "adc_drv.h"
#include "adc_grp_help.h"
#include "sensor_def.h"

extern const mp_obj_type_t adc_group_type;

#define MAX_GROUP_SIZE      8
#define MAX_GROUP_NUMBER    2

typedef struct _adc_grp_obj_t {
    mp_obj_base_t base;

    ADC_Type *  adc_base;
    uint8_t     adc_id;
    uint8_t     adc_channel[MAX_GROUP_SIZE];

    mp_obj_array_t  ret_obj;
    uint16_t    ret_array[MAX_GROUP_SIZE];

    uint16_t    captured_value[MAX_GROUP_SIZE];
} adc_grp_obj_t;
adc_grp_obj_t ADC_Group[MAX_GROUP_NUMBER];

STATIC void adc_grp_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    adc_grp_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "ADC_Group: %d ADC pins", self->ret_obj.len);
}

/// \classmethod \constructor()
/// Create and return an ADC_Group object.
/// Construct an object with ID and an array of ADC objects.
///
/// ADC_Group(id, group)
STATIC mp_obj_t adc_grp_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{
    enum { ARG_id, ARG_period, ARG_average};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_id, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_period, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = kADC_SamplePeriodLong24Clcoks}},
        { MP_QSTR_average, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = kADC_HardwareAverageCount16}},
    };

    // Parse args.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int adc_grp_id = args[ARG_id].u_int;
    if (adc_grp_id < 1 || adc_grp_id > MAX_GROUP_NUMBER)
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("ID must be 1~%d."), MAX_GROUP_NUMBER);

    ADC_Type *ADC_base[] = ADC_BASE_PTRS;
    adc_grp_obj_t *self = &ADC_Group[adc_grp_id-1];
    memset(self, 0, sizeof(adc_grp_obj_t));
    self->base.type = &adc_group_type;
    self->adc_id = adc_grp_id;
    self->adc_base = ADC_base[adc_grp_id];

    self->ret_obj.base.type = &mp_type_bytearray;
    self->ret_obj.typecode = 'H';   // Unsigned short
    self->ret_obj.free = 0;
    self->ret_obj.len = 0;
    self->ret_obj.items = self->ret_array;

    int period = args[ARG_period].u_int;
    int average= args[ARG_average].u_int;
    adc_hw_init(adc_grp_id, period, average);
    return MP_OBJ_FROM_PTR(self);
}

/// \classmethod addch(ch1, ch2, ....)
/// Add a channel to the scan group
STATIC mp_obj_t adc_grp_addch(mp_obj_t self_in, mp_obj_t chn_in)
{
    adc_grp_obj_t *self = MP_OBJ_TO_PTR(self_in);

    const machine_pin_obj_t *pin = pin_find(chn_in);
    int ilist;
    for (ilist = 0; ilist < pin->adc_list_len; ilist++) {
        if (pin->adc_list[ilist].instance == self->adc_base)
            break;
    }
    if (ilist == pin->adc_list_len)
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("Pin(%q) is not an ADC%d pin"), pin->name, self->adc_id);

    int idx = self->ret_obj.len;
    int chn = pin->adc_list[ilist].channel;
    for (ilist = 0; ilist < idx; ilist++) {
        if (chn == self->adc_channel[ilist])
            mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("Pin(%q) is in the group."), pin->name);
    }
    self->ret_obj.len++;
    self->adc_channel[idx] = chn;
    scan_adc_enabled(self->adc_base, self->adc_channel[idx] | 0xF0/*Scan mode*/, true/*Enable*/);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(adc_grp_addch_obj, adc_grp_addch);

void fill_array(adc_grp_obj_t *self)
{
    for (int idx = 0; idx < self->ret_obj.len; idx++)
        self->ret_array[idx] = scan_adc_convert(self->adc_base, self->adc_channel[idx]);
}

STATIC mp_uint_t protocol_read(mp_obj_t obj, void *buf, mp_uint_t size)
{
    adc_grp_obj_t *self = MP_OBJ_TO_PTR(obj);
    uint32_t *result = (uint32_t*)buf;
    int num = MIN(self->ret_obj.len, size);
    for (int idx = 0; idx < num; idx++)
        result[idx] = scan_adc_convert(self->adc_base, self->adc_channel[idx]);
    return num;
}

/// \classmethod read()
STATIC mp_obj_t adc_grp_read(mp_obj_t self_in)
{
    adc_grp_obj_t *self = MP_OBJ_TO_PTR(self_in);
    fill_array(self);
    return MP_OBJ_FROM_PTR(&self->ret_obj);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(adc_grp_read_obj, adc_grp_read);

/// \classmethod capture()
/// Read and save the values
STATIC mp_obj_t adc_grp_capture(mp_obj_t self_in)
{
    adc_grp_obj_t *self = MP_OBJ_TO_PTR(self_in);
    for (int idx = 0; idx < self->ret_obj.len; idx++)
        self->captured_value[idx] = scan_adc_convert(self->adc_base, self->adc_channel[idx]);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(adc_grp_capture_obj, adc_grp_capture);

STATIC mp_uint_t protocol_capture(mp_obj_t obj)
{
    adc_grp_obj_t *self = MP_OBJ_TO_PTR(obj);
    adc_grp_capture(obj);
    return self->ret_obj.len;
}

/// \classmethod get()
/// Read the saved values
STATIC mp_obj_t adc_grp_get(mp_obj_t self_in)
{
    adc_grp_obj_t *self = MP_OBJ_TO_PTR(self_in);
    memcpy(self->ret_array, self->captured_value, sizeof(self->captured_value));
    return MP_OBJ_FROM_PTR(&self->ret_obj);;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(adc_grp_get_obj, adc_grp_get);

STATIC mp_uint_t protocol_get(mp_obj_t obj, void *buf, mp_uint_t size)
{
    adc_grp_obj_t *self = MP_OBJ_TO_PTR(obj);
    uint32_t *result = (uint32_t*)buf;
    int num = MIN(self->ret_obj.len, size);
    for (int idx = 0; idx < self->ret_obj.len; idx++)
        result[idx] = self->ret_array[idx];
    return num;
}

STATIC mp_obj_t adc_grp_help(size_t n_args, const mp_obj_t *args) {
    if (n_args == 0)
        mp_print_str(MP_PYTHON_PRINTER, ADC_GRP_HELP);
    else {
        int help_id = mp_obj_get_int(args[0]);
        char *help_text[ADC_GRP_LAST_HELP] = {
            ADC_GRP_NEW_HELP_TEXT,
            ADC_GRP_ADDCH_HELP_TEXT,
            ADC_GRP_CAPTURE_HELP_TEXT,
            ADC_GRP_READ_HELP_TEXT,
            ADC_GRP_GET_HELP_TEXT,
        };
        if (help_id >= 0 && help_id < ADC_GRP_LAST_HELP)
            mp_print_str(MP_PYTHON_PRINTER, help_text[help_id]);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(adc_grp_help_func, 0, 1, adc_grp_help);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(adc_grp_help_obj, MP_ROM_PTR(&adc_grp_help_func));


STATIC const mp_rom_map_elem_t adc_grp_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_help),    MP_ROM_PTR(&adc_grp_help_obj) },

    { MP_ROM_QSTR(MP_QSTR_addch),  MP_ROM_PTR(&adc_grp_addch_obj) },
    { MP_ROM_QSTR(MP_QSTR_capture),MP_ROM_PTR(&adc_grp_capture_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),   MP_ROM_PTR(&adc_grp_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_get),    MP_ROM_PTR(&adc_grp_get_obj) },

    // Keywords for constructor 'period'
    { MP_ROM_QSTR(MP_QSTR_PMODE0), MP_ROM_INT(kADC_SamplePeriodLong12Clcoks) },
    { MP_ROM_QSTR(MP_QSTR_PMODE1), MP_ROM_INT(kADC_SamplePeriodLong16Clcoks) },
    { MP_ROM_QSTR(MP_QSTR_PMODE2), MP_ROM_INT(kADC_SamplePeriodLong20Clcoks) },
    { MP_ROM_QSTR(MP_QSTR_PMODE3), MP_ROM_INT(kADC_SamplePeriodLong24Clcoks) },

    // Keywords for constructor 'average'
    { MP_ROM_QSTR(MP_QSTR_AVG1),    MP_ROM_INT(kADC_HardwareAverageDiasable) },
    { MP_ROM_QSTR(MP_QSTR_AVG4),    MP_ROM_INT(kADC_HardwareAverageCount4) },
    { MP_ROM_QSTR(MP_QSTR_AVG8),    MP_ROM_INT(kADC_HardwareAverageCount8) },
    { MP_ROM_QSTR(MP_QSTR_AVG16),   MP_ROM_INT(kADC_HardwareAverageCount16) },
    { MP_ROM_QSTR(MP_QSTR_AVG32),   MP_ROM_INT(kADC_HardwareAverageCount32) },
};
STATIC MP_DEFINE_CONST_DICT(adc_grp_locals_dict, adc_grp_locals_dict_table);

STATIC const sensor_protocol_t adc_grp_p = {
    .name =     MP_QSTR_sensor,
    .read =     protocol_read,      // Read the values
    .get =      protocol_get,       // Get the saved values
    .capture =  protocol_capture,   // Read and save the values
};


MP_DEFINE_CONST_OBJ_TYPE(
    adc_group_type,
    MP_QSTR_ADC_Group,
    MP_TYPE_FLAG_NONE,
    make_new, adc_grp_make_new,
    print, adc_grp_print,
    protocol, &adc_grp_p,
    locals_dict, &adc_grp_locals_dict
    );
