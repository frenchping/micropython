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
#include "fsl_iomuxc.h"
#include "fsl_qtmr.h"

#include "sensor_def.h"

extern const mp_obj_type_t encoder_type;
typedef struct _smartcar_encoder_obj_t {
    mp_obj_base_t base;
    const machine_pin_obj_t *phaseA;
    const machine_pin_obj_t *phaseB;
    TMR_Type * TMR_base;
    bool    invert;     // If the return value should be inverted
    uint8_t pinA_id;
    uint8_t pinB_id;
    int     saved_val;
} smartcar_encoder_obj_t;

#define ENCODER_PIN_CONF     (IOMUXC_SW_PAD_CTL_PAD_SPEED(1) \
                                | IOMUXC_SW_PAD_CTL_PAD_PKE(1) \
                                | IOMUXC_SW_PAD_CTL_PAD_PUS(0) \
                                | IOMUXC_SW_PAD_CTL_PAD_DSE(6))  // 100MHz, 100K PD, Keeper, R/6

#define TMR_TIMER(id, idx)    MP_QSTR_TMR##id##_TIMER##idx
#define TMR_INPUT(id)   { TMR_TIMER(id, 0), TMR_TIMER(id, 1), TMR_TIMER(id, 2), TMR_TIMER(id, 3) }
static const qstr Tmr_Input[FSL_FEATURE_SOC_TMR_COUNT][4] = {
    TMR_INPUT(1),
    TMR_INPUT(2),
#if FSL_FEATURE_SOC_TMR_COUNT > 2
    TMR_INPUT(3),
#if FSL_FEATURE_SOC_TMR_COUNT > 3
    TMR_INPUT(4)
#endif
#endif
};

STATIC void encoder_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    smartcar_encoder_obj_t *self = MP_OBJ_TO_PTR(self_in);
    // mp_printf(print, "Encoder: (PinA=%q(%q), PinB=%q(%q), saved_value=%d, %s)",
    //         self->phaseA->name, self->phaseA->board_name,
    //         self->phaseB->name, self->phaseB->board_name, self->saved_val,
    //         self->invert ? "inverted" : "not inverted");
    mp_printf(print, "Encoder: (PinA=%q, PinB=%q, saved_value=%d, %s)",
            self->phaseA->name,
            self->phaseB->name, self->saved_val,
            self->invert ? "inverted" : "not inverted");
}

/// \classmethod \constructor()
/// Create and return an encoder object.
///
/// Construct a encoder object with given 2 pins.
///
/// encoder(PhaseA_pin, PhaseB_pin [, invert = False])
STATIC mp_obj_t encoder_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    extern const machine_pin_af_obj_t *pin_find_af_by_base(const machine_pin_obj_t *pin, void *base_ptr[], size_t base_size);
    mp_arg_check_num(n_args, n_kw, 2, 3, false);

    const machine_pin_obj_t *pinA = pin_find(args[0]);
    const machine_pin_obj_t *pinB = pin_find(args[1]);
    if (!pinA || !pinB)
        mp_raise_ValueError(MP_ERROR_TEXT("provide A,B pins"));
    
    static TMR_Type* tmr_base[] = TMR_BASE_PTRS;
    const machine_pin_af_obj_t *pinA_af = pin_find_af_by_base(pinA, (void**)&tmr_base[1], FSL_FEATURE_SOC_TMR_COUNT);
    const machine_pin_af_obj_t *pinB_af = pin_find_af_by_base(pinB, (void**)&tmr_base[1], FSL_FEATURE_SOC_TMR_COUNT);
    if (pinA_af == NULL || pinB_af == NULL)
        mp_raise_ValueError(MP_ERROR_TEXT("either A,B pins must be TMR pin."));

    if (pinA_af->instance != pinB_af->instance)
        mp_raise_ValueError(MP_ERROR_TEXT("A,B pins must from same TMR instance."));
    TMR_Type * TMR_base = pinA_af->instance;
    // Find out the TMR instance id
    int tmr_id;
    for (tmr_id = 1; tmr_id <= FSL_FEATURE_SOC_TMR_COUNT; tmr_id++)
        if (tmr_base[tmr_id] == TMR_base)
            break;
    // tmr_id must be valid on this point. No need to validate again.

    tmr_id--;
    // Find out TMR_INPUT pin number
    int pinA_id, pinB_id;
    for (pinA_id = 0; pinA_id < 4; pinA_id++)
        if (Tmr_Input[tmr_id][pinA_id] == pinA_af->name)
            break;
    for (pinB_id = 0; pinB_id < 4; pinB_id++)
        if (Tmr_Input[tmr_id][pinB_id] == pinB_af->name)
            break;
    // Both pin_id are valid on this point.

    smartcar_encoder_obj_t *self = m_new_obj(smartcar_encoder_obj_t);
    memset(self, 0, sizeof(smartcar_encoder_obj_t));
    self->base.type = &encoder_type;

    // Config both pins.
    IOMUXC_SetPinMux(pinA->muxRegister,     pinA_af->af_mode, pinA_af->input_register, pinA_af->input_daisy, pinA->configRegister, 1U);
    IOMUXC_SetPinConfig(pinA->muxRegister,  pinA_af->af_mode, pinA_af->input_register, pinA_af->input_daisy, pinA->configRegister, ENCODER_PIN_CONF);
    IOMUXC_SetPinMux(pinB->muxRegister,     pinB_af->af_mode, pinB_af->input_register, pinB_af->input_daisy, pinB->configRegister, 1U);
    IOMUXC_SetPinConfig(pinB->muxRegister,  pinB_af->af_mode, pinB_af->input_register, pinB_af->input_daisy, pinB->configRegister, ENCODER_PIN_CONF);

    self->phaseA = pinA;
    self->phaseB = pinB;
    self->pinA_id = pinA_id;
    self->pinB_id = pinB_id;
    self->TMR_base = TMR_base;
    {   // qtimer_quad_init()
        qtmr_config_t qtmrConfig;
        QTMR_GetDefaultConfig(&qtmrConfig);
        qtmrConfig.primarySource = (qtmr_primary_count_source_t)(pinA_id);
        qtmrConfig.secondarySource = (qtmr_input_source_t)(pinB_id);

        QTMR_Init(TMR_base, (qtmr_channel_selection_t)(pinA_id), &qtmrConfig);//第一次初始化便于打开时钟
        QTMR_Deinit(TMR_base, (qtmr_channel_selection_t)(pinA_id));           //复位外设
        QTMR_Init(TMR_base, (qtmr_channel_selection_t)(pinA_id), &qtmrConfig);//重新初始化设置正确的参数
        
        QTMR_StartTimer(TMR_base, (qtmr_channel_selection_t)(pinA_id), kQTMR_PriSrcRiseEdgeSecDir);        
    }

    self->invert = false;
    if (n_args == 3)
        self->invert = mp_obj_is_true(args[2]);
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_uint_t protocol_read(mp_obj_t obj, void *buf, mp_uint_t size)
{
    smartcar_encoder_obj_t *self = MP_OBJ_TO_PTR(obj);
    int *result = (int*)buf;
    int value = QTMR_GetCurrentTimerCount(self->TMR_base, (qtmr_channel_selection_t)(self->pinA_id));
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
    TMR_Type *tmr = (TMR_Type *)self->TMR_base;
    int value = QTMR_GetCurrentTimerCount(tmr, (qtmr_channel_selection_t)(self->pinA_id));
    if (value & 0x8000)
        value -= 65536;
    tmr->CHANNEL[self->pinA_id].CNTR = 0;
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

MP_DEFINE_CONST_OBJ_TYPE(
    encoder_type,
    MP_QSTR_encoder,
    MP_TYPE_FLAG_NONE,
    make_new, encoder_make_new,
    print, encoder_print,
    protocol, &encoder_p,
    locals_dict, &encoder_locals_dict
    );
