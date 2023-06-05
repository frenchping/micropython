/*
 * Encoder_py.c
 *
 *  Created on: Dec 16, 2021
 *      Author: nxa19017
 * 
 *  Micropython functions for manipulate QTimer/Encoder
 *  This module superseed functions in zfp_qtimer.c
 * 
 */
#include "py/runtime.h"
#include "mphalport.h"
#include "fsl_qtmr.h"
#include "cfg_mux_mgr.h"

#include "Sensor_def.h"

extern const mp_obj_type_t smartcar_encoder_type;
typedef struct _smartcar_encoder_obj_t {
    mp_obj_base_t base;
    pin_obj_t *phaseA;
    pin_obj_t *phaseB;
    int     saved_val;
    bool    invert;     // If the return value should be inverted
} smartcar_encoder_obj_t;

#define ENCODER_PIN_CONF     (IOMUXC_SW_PAD_CTL_PAD_SPEED(1) \
                                | IOMUXC_SW_PAD_CTL_PAD_PKE(1) \
                                | IOMUXC_SW_PAD_CTL_PAD_PUS(0) \
                                | IOMUXC_SW_PAD_CTL_PAD_DSE(6))  // 100MHz, 100K PD, Keeper, R/6

#ifdef MIMXRT1020_SERIES
struct {
    pin_af_obj_t *af;
    uint8_t type;
} af_type;
#else
#error "QTimer Pin-map is not defined."
#endif


STATIC void encoder_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    smartcar_encoder_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "Encoder: (PinA=%q(%q), PinB=%q(%q), saved_value=%d, %s)",
            self->phaseA->name, self->phaseA->board_name,
            self->phaseB->name, self->phaseB->board_name, self->saved_val,
            self->invert ? "inverted" : "not inverted");
}

/// \classmethod \constructor()
/// Create and return an encoder object.
///
/// Construct a encoder object with given UART object.
///
/// encoder(uart)
STATIC mp_obj_t encoder_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 1, 2, false);

    int id_num = mp_obj_get_int(args[0]);
    MuxItem_t PinA, PinB;
    Mux_Query(mp_const_none, "encoder", id_num, "PA", &PinA);
    Mux_Query(mp_const_none, "encoder", id_num, "PB", &PinB);
    if (!PinA.pPinObj || !PinB.pPinObj)
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("encoder(%d) is not defined in cmm_cfg.csv"), id_num);

    smartcar_encoder_obj_t *self = m_new_obj(smartcar_encoder_obj_t);
    memset(self, 0, sizeof(smartcar_encoder_obj_t));
    self->base.type = &smartcar_encoder_type;

    Mux_Take(self, "encoder", id_num, "PA", &PinA);
    Mux_Take(self, "encoder", id_num, "PB", &PinB);
    mp_hal_pin_config_alt(PinA.pPinObj, ENCODER_PIN_CONF, AF_FN_QTIMER);
    mp_hal_pin_config_alt(PinB.pPinObj, ENCODER_PIN_CONF, AF_FN_QTIMER);

    self->phaseA = (pin_obj_t*)PinA.pPinObj;
    self->phaseB = (pin_obj_t*)PinB.pPinObj;
    {   // qtimer_quad_init()
        qtmr_config_t qtmrConfig;
        QTMR_GetDefaultConfig(&qtmrConfig);
        qtmrConfig.primarySource = (qtmr_primary_count_source_t)(self->phaseA->af->af_pin_type);
        qtmrConfig.secondarySource = (qtmr_input_source_t)(self->phaseB->af->af_pin_type);

        QTMR_Init(self->phaseA->af->reg, (qtmr_channel_selection_t)(self->phaseA->af->af_pin_type), &qtmrConfig);//第一次初始化便于打开时钟
        QTMR_Deinit(self->phaseA->af->reg, (qtmr_channel_selection_t)(self->phaseA->af->af_pin_type));           //复位外设
        QTMR_Init(self->phaseA->af->reg, (qtmr_channel_selection_t)(self->phaseA->af->af_pin_type), &qtmrConfig);//重新初始化设置正确的参数
        
        QTMR_StartTimer(self->phaseA->af->reg, (qtmr_channel_selection_t)(self->phaseA->af->af_pin_type), kQTMR_PriSrcRiseEdgeSecDir);        
    }

    self->invert = false;
    if (n_args == 2)
        self->invert = mp_obj_is_true(args[1]);
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_uint_t protocol_read(mp_obj_t obj, void *buf, mp_uint_t size)
{
    smartcar_encoder_obj_t *self = MP_OBJ_TO_PTR(obj);
    int *result = (int*)buf;
    int value = QTMR_GetCurrentTimerCount(self->phaseA->af->reg, (qtmr_channel_selection_t)(self->phaseA->af->af_pin_type));
    if (value & 0x8000)
        value -= 65536;
    *result = (self->invert ? -value : value);
    return 1;
}
STATIC mp_obj_t encoder_read(mp_obj_t self_in)
{
    int value;
    protocol_read(self_in, &value, 1);
    return MP_OBJ_NEW_SMALL_INT(value);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(encoder_read_obj, encoder_read);

STATIC mp_uint_t protocol_capture(mp_obj_t obj)
{
    smartcar_encoder_obj_t *self = MP_OBJ_TO_PTR(obj);
    TMR_Type *tmr = (TMR_Type *)self->phaseA->af->reg;
    int value = QTMR_GetCurrentTimerCount(tmr, (qtmr_channel_selection_t)(self->phaseA->af->af_pin_type));
    if (value & 0x8000)
        value -= 65536;
    tmr->CHANNEL[self->phaseA->af->af_pin_type].CNTR = 0;
    self->saved_val = (self->invert ? -value : value);
    return 1;
}
STATIC mp_obj_t encoder_capture(mp_obj_t self_in)
{
    protocol_capture(self_in);
    smartcar_encoder_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return MP_OBJ_NEW_SMALL_INT(self->saved_val);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(encoder_capture_obj, encoder_capture);

STATIC mp_uint_t protocol_get(mp_obj_t obj, void *buf, mp_uint_t size)
{
    smartcar_encoder_obj_t *self = MP_OBJ_TO_PTR(obj);
    int *result = (int*)buf;
    *result = self->saved_val;
    return 1;
}
STATIC mp_obj_t encoder_get(mp_obj_t self_in)
{
    smartcar_encoder_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return MP_OBJ_NEW_SMALL_INT(self->saved_val);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(encoder_get_obj, encoder_get);

STATIC const mp_rom_map_elem_t encoder_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_get),     MP_ROM_PTR(&encoder_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),    MP_ROM_PTR(&encoder_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_capture), MP_ROM_PTR(&encoder_capture_obj) },
};

STATIC MP_DEFINE_CONST_DICT(encoder_locals_dict, encoder_locals_dict_table);

STATIC const sensor_protocol_t encoder_p = {
    .name =     MP_QSTR_Sensor,
    .read =     protocol_read,      // Read the values
    .get =      protocol_get,       // Get the saved values
    .capture =  protocol_capture,   // Read and save the values
};

const mp_obj_type_t smartcar_encoder_type = {
    { &mp_type_type },
    .name = MP_QSTR_Encoder,
    .make_new = encoder_make_new,
    .protocol = &encoder_p,
    .print = encoder_print,
    .locals_dict = (mp_obj_dict_t *)&encoder_locals_dict,
};
