/*
 * PWM_py.c
 *
 *  Created on: Dec 16, 2021
 *      Author: nxa19017
 * 
 *  Micropython functions for manipulate PWM & Motor
 *  This module superseed functions in zfp_pwm.c
 * 
 */


#include "py/runtime.h"
#include "mphalport.h"
#include "fsl_pwm.h"
#include "cfg_mux_mgr.h"

#define PWM_CHANNEL_NUM     8       // Number of output pins
#define PWM_FREQUENCE_MIN   50      //
#define PWM_FREQUENCE_MAX   50000   //
#define PWM_DEFAULT_FREQ0   13564   // with this freq, the duty grid is 2400h
#define PWM_DEFAULT_FREQ    15259   // with this freq, the duty grid is 2000h

#define PWM_MAX_DUTY        100000L

extern const mp_obj_type_t smartcar_pwm_type;
extern const mp_obj_type_t smartcar_pwmch_type;

#define PIN_SUBMODULE(pin_obj)  (pin_obj->af->af_pin_type / 3)  // See pin_defs_mcu.h AF_PIN_TYPE_PWM_PWMA0 ....
#define PIN_CHANNEL(pin_obj)    ((pin_obj->af->af_pin_type % 3) ? kPWM_PwmB : kPWM_PwmA)

typedef struct _smartcar_pwmch_obj_t smartcar_pwmch_obj_t;  // Predefine

typedef struct _smartcar_pwm_obj_t {
    mp_obj_base_t base;

    PWM_Type    *pwm_base;
    pwm_config_t pwmConfig;

    uint8_t     pwm_id;         // Either 1 or 2
    uint16_t    freq;           // Frequence
    smartcar_pwmch_obj_t *channels[PWM_CHANNEL_NUM];
} smartcar_pwm_obj_t;

typedef struct _pwm_pin_obj_t {
    const pin_obj_t *pin_obj;
    uint32_t    duty;
    uint16_t    freq;
    pwm_submodule_t subm;
    pwm_channels_t  side;
} pwm_pin_obj_t;

typedef enum _pwmch_type {
    TYPE_SINGAL_CHNNEL = 0,
    TYPE_DOUBLE_CHNNEL,
    TYPE_PWM_GPIO_CHNNEL,
} pwmch_type;

typedef struct _smartcar_pwmch_obj_t {
    mp_obj_base_t base;
    smartcar_pwm_obj_t  *pwm_obj;
    pwmch_type type;
    pwm_pin_obj_t pos;
    pwm_pin_obj_t neg;
} smartcar_pwmch_obj_t;

#define PWM_PIN_CONF    (IOMUXC_SW_PAD_CTL_PAD_SPEED(1) \
                                | IOMUXC_SW_PAD_CTL_PAD_PKE(1) \
                                | IOMUXC_SW_PAD_CTL_PAD_PUS(0) \
                                | IOMUXC_SW_PAD_CTL_PAD_DSE(6))  // 100MHz, 100K PD, Keeper, R/6
#define PWM_SRC_CLK_FREQ    CLOCK_GetFreq(kCLOCK_IpgClk)        //定义PWM输入时钟源频率

static void init_pwm_config(smartcar_pwm_obj_t *self)
{
    uint16_t temp_prsc;
    PWM_GetDefaultConfig(&self->pwmConfig);
    self->pwmConfig.reloadLogic = kPWM_ReloadPwmFullCycle;
    self->pwmConfig.pairOperation = kPWM_Independent;
    self->pwmConfig.enableDebugMode = true;

    //计算分频系数
    uint32_t period = PWM_SRC_CLK_FREQ / self->freq;
    temp_prsc = period >> 16;
    if (period & 0xFFFF)
        temp_prsc++;        // Round up
    self->pwmConfig.prescale = (1   >= temp_prsc) ? kPWM_Prescale_Divide_1 :
                                (2   >= temp_prsc) ? kPWM_Prescale_Divide_2 :
                                (4   >= temp_prsc) ? kPWM_Prescale_Divide_4 :
                                (8   >= temp_prsc) ? kPWM_Prescale_Divide_8 :
                                (16  >= temp_prsc) ? kPWM_Prescale_Divide_16 :
                                (32  >= temp_prsc) ? kPWM_Prescale_Divide_32 :
                                (64  >= temp_prsc) ? kPWM_Prescale_Divide_64 :
                                (128 >= temp_prsc) ? kPWM_Prescale_Divide_128 :
                (mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Freq is too low or IPG is too high.")), -1);    //频率过小 或者IPG频率过高
}

/// \classmethod \constructor()
/// Create and return an pwm object.
///
/// Construct a pwm object with ID-num and optional a given frequence.
///
/// PWM(id, *, freq)
STATIC mp_obj_t pwm_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 1, 2, false);

    int pwm_id = mp_obj_get_int(args[0]);
    if (pwm_id != 1 && pwm_id != 2)
        mp_raise_ValueError("ID must be 1 or 2");

    smartcar_pwm_obj_t *self = m_new_obj(smartcar_pwm_obj_t);
    memset(self, 0, sizeof(smartcar_pwm_obj_t));
    self->base.type = &smartcar_pwm_type;
    self->pwm_id = pwm_id;
    self->pwm_base = (pwm_id == 1) ? PWM1 : PWM2;
    if (n_args == 2)
        self->freq = mp_obj_get_int(args[1]);
    else
        self->freq = PWM_DEFAULT_FREQ;

    // Initialize the configuration structure.
    init_pwm_config(self);
    return MP_OBJ_FROM_PTR(self);
}

/// \classmethod channel(chn, pinA, [pinB])
/// Define a channel, either single channel or a pair channel
/// chn has to be continuously, otherwise it is unpredictable
STATIC mp_obj_t pwm_channel(size_t n_args, const mp_obj_t *args)
{
    smartcar_pwm_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int chn = mp_obj_get_int(args[1]);
    if (chn >= PWM_CHANNEL_NUM)
        mp_raise_ValueError("too many channels.");

    smartcar_pwmch_obj_t *pwmch = m_new_obj(smartcar_pwmch_obj_t);
    memset(pwmch, 0, sizeof(smartcar_pwmch_obj_t));
    pwmch->base.type = &smartcar_pwmch_type;
    pwmch->pwm_obj = self;

    const char *pin_name = (char*)mp_obj_str_get_str(args[2]);
    MuxItem_t PinA, PinB;
    Mux_Take(self, "pwm", self->pwm_id, pin_name, &PinA);
    if (PinA.pPinObj == NULL || PinA.pPinObj == mp_const_none)
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("%s is not defined in cmm_cfg.csv"), pin_name);
    mp_hal_pin_config_alt(PinA.pPinObj, PWM_PIN_CONF, AF_FN_PWM);
    pwmch->pos.pin_obj = PinA.pPinObj;
    pwmch->pos.subm = PIN_SUBMODULE(PinA.pPinObj);
    pwmch->pos.side = PIN_CHANNEL(PinA.pPinObj);
    pwmch->pos.freq = self->freq;
    pwmch->type = TYPE_SINGAL_CHNNEL;

    if (n_args == 4) {
        if (mp_obj_is_type(args[3], &pyb_pin_type)) {
            // the 2nd pin is GPIO for direction
            pwmch->neg.pin_obj = (pin_obj_t*)args[3];
            pwmch->type = TYPE_PWM_GPIO_CHNNEL;
        }
        else {
            pin_name = mp_obj_str_get_str(args[3]);
            Mux_Take(self, "pwm", self->pwm_id, pin_name, &PinB);
            if (PinB.pPinObj == NULL)
                mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("%s is not defined in cmm_cfg.csv"), pin_name);
            mp_hal_pin_config_alt(PinB.pPinObj, PWM_PIN_CONF, AF_FN_PWM);
            pwmch->neg.pin_obj = PinB.pPinObj;
            pwmch->neg.subm = PIN_SUBMODULE(PinB.pPinObj);
            pwmch->neg.side = PIN_CHANNEL(PinB.pPinObj);
            pwmch->neg.freq = self->freq;
            pwmch->type = TYPE_DOUBLE_CHNNEL;
        }
    }
    self->channels[chn-1] = pwmch;
    return MP_OBJ_FROM_PTR(pwmch);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pwm_channel_obj, 3, 4, pwm_channel);

uint16_t pwmHighPulse;
static void update_duty(smartcar_pwm_obj_t *self, pwm_pin_obj_t *pwm_pin, int duty)
{
    //清除LOAD OKAY位  以设置新的参数
    PWM_SetPwmLdok(self->pwm_base, (pwm_module_control_t)(1 << pwm_pin->subm), false);
    PWM_UpdatePwmDutycycle(self->pwm_base, pwm_pin->subm,
                                pwm_pin->side, kPWM_EdgeAligned, duty/(PWM_MAX_DUTY/100));

    {   // Recalculate to have better accuracy
        uint16_t VAL1 = self->pwm_base->SM[pwm_pin->subm].VAL1;
        uint16_t pwmHighPulse = (uint32_t)(VAL1 * duty) / PWM_MAX_DUTY;
        if (pwm_pin->side == kPWM_PwmA) {
            // self->pwm_base->SM[pwm_pin->subm].VAL2 = 0;
            self->pwm_base->SM[pwm_pin->subm].VAL3 = pwmHighPulse;
        }
        else {
            // self->pwm_base->SM[pwm_pin->subm].VAL4 = 0;
            self->pwm_base->SM[pwm_pin->subm].VAL5 = pwmHighPulse;
        }
    }
    //设置LOAD OKAY位  以更新设置
    PWM_SetPwmLdok(self->pwm_base, (pwm_module_control_t)(1 << pwm_pin->subm), true);
    pwm_pin->duty = duty;        // Remember it
}

static void chn_init(smartcar_pwm_obj_t *self, pwm_pin_obj_t *pwm_pin)
{
    PWM_Type    *pwm_base = self->pwm_base;
    if (PWM_Init(pwm_base, pwm_pin->subm, &self->pwmConfig) == kStatus_Fail)    // 第一次初始化便于打开时钟
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Error on 1st PWM_Init()"));
    PWM_Deinit(pwm_base, pwm_pin->subm);
    if (PWM_Init(pwm_base, pwm_pin->subm, &self->pwmConfig) == kStatus_Fail)    // 重新初始化设置正确的参数
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Error on 2nd PWM_Init()"));

    //设置频率占空比等参数
    pwm_signal_param_t  pwmSignal;
    pwmSignal.pwmChannel = pwm_pin->side; 
    pwmSignal.level = kPWM_HighTrue;
    pwmSignal.dutyCyclePercent = pwm_pin->duty;  // Was init as 0
    pwmSignal.deadtimeValue = 0;        //((uint64_t)PWM_SRC_CLK_FREQ * 650) / 1000000000;

    //清除LOAD OKAY位  以设置新的参数
    PWM_SetPwmLdok(pwm_base, (pwm_module_control_t)(1 << pwm_pin->subm), false);
    PWM_SetupPwm(pwm_base, (pwm_submodule_t)(pwm_pin->subm), &pwmSignal, 1, kPWM_EdgeAligned, self->freq, PWM_SRC_CLK_FREQ);     
    //设置LOAD OKAY位  以更新设置
    PWM_SetPwmLdok(pwm_base, (pwm_module_control_t)(1 << pwm_pin->subm), true);

    //启动定时器
    PWM_StartTimer(pwm_base, (pwm_module_control_t)(1 << pwm_pin->subm));
    
    pwm_base->SM[pwm_pin->subm].DISMAP[0]=0;
}

STATIC void pwmch_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    smartcar_pwmch_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    mp_printf(print, "PWM ID=%d\tChn Type=%d\r\n", self->pwm_obj->pwm_id, self->type);
    mp_printf(print, "\tPositive Pin: name='%q', side=%s, subm=%d, freq=%d, duty=%d\r\n",
                self->pos.pin_obj->board_name,
                (self->pos.side == kPWM_PwmA) ? "kPWM_PwmA" : "kPWM_PwmB", self->pos.subm,
                self->pos.freq, self->pos.duty);
    if (self->type == TYPE_DOUBLE_CHNNEL) {
        mp_printf(print, "\tNegative Pin: name='%q', side=%s, subm=%d, freq=%d, duty=%d\r\n",
                self->neg.pin_obj->board_name,
                (self->neg.side == kPWM_PwmA) ? "kPWM_PwmA" : "kPWM_PwmB", self->neg.subm,
                self->neg.freq, self->neg.duty);
    }
    if (self->type == TYPE_PWM_GPIO_CHNNEL)
        mp_printf(print, "\tGPIO pin: name='%q'\r\n", self->neg.pin_obj->board_name);
}

/// \classmethod PWMch.freq(value)
STATIC mp_obj_t pwmch_freq(mp_obj_t self_in, mp_obj_t freq_in)
{
    smartcar_pwmch_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int freq = mp_obj_get_int(freq_in);
    if (freq < PWM_FREQUENCE_MIN || freq > PWM_FREQUENCE_MAX)
        mp_raise_ValueError("out of range.");
    self->pos.freq = freq;
    self->neg.freq = freq;
    self->pwm_obj->freq = freq;
    init_pwm_config(self->pwm_obj);
    chn_init(self->pwm_obj, &self->pos);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pwmch_freq_obj, pwmch_freq);

/// \classmethod PWMch.duty(value)
STATIC mp_obj_t pwmch_duty(mp_obj_t self_in, mp_obj_t duty_in)
{
    smartcar_pwmch_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int duty;
    if (mp_obj_is_float(duty_in))
        duty = mp_obj_get_float(duty_in);
    else
        duty = mp_obj_get_int(duty_in);
    if (duty > PWM_MAX_DUTY)
        duty = PWM_MAX_DUTY;
    else if (duty < -PWM_MAX_DUTY)
        duty = -PWM_MAX_DUTY;

    if (duty >= 0) {
        update_duty(self->pwm_obj, &self->pos, duty);
        if (self->type == TYPE_DOUBLE_CHNNEL)
            update_duty(self->pwm_obj, &self->neg, 0);
        else if (self->type == TYPE_PWM_GPIO_CHNNEL)
            mp_hal_pin_write(self->neg.pin_obj, 1);
    }
    else {
        if (self->type == TYPE_PWM_GPIO_CHNNEL) {
            update_duty(self->pwm_obj, &self->pos, -duty);
            mp_hal_pin_write(self->neg.pin_obj, 0);
        }
        else {
            update_duty(self->pwm_obj, &self->pos, 0);
            if (self->type == TYPE_DOUBLE_CHNNEL)
                update_duty(self->pwm_obj, &self->neg, -duty);
        }
    }
    return MP_OBJ_NEW_SMALL_INT(duty);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pwmch_duty_obj, pwmch_duty);

/// \classmethod start()
/// Running the PWM Timer
STATIC mp_obj_t pwm_start(mp_obj_t self_in)
{
    smartcar_pwm_obj_t *self = MP_OBJ_TO_PTR(self_in);

    // 初始化PWM模块
    for (int chn = 0; chn < PWM_CHANNEL_NUM; chn++) {   // Initialize all channels
        smartcar_pwmch_obj_t *chn_obj = self->channels[chn];
        if (!chn_obj)
        	continue;
        chn_init(self, &chn_obj->pos);
        if (chn_obj->type == TYPE_DOUBLE_CHNNEL)
            chn_init(self, &chn_obj->neg);
        else if (chn_obj->type == TYPE_PWM_GPIO_CHNNEL)
            mp_hal_pin_write(chn_obj->neg.pin_obj, 1);  // Start with positive direction
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pwm_start_obj, pwm_start);

STATIC const mp_rom_map_elem_t pwm_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_channel), MP_ROM_PTR(&pwm_channel_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),   MP_ROM_PTR(&pwm_start_obj) },
};

STATIC MP_DEFINE_CONST_DICT(pwm_locals_dict, pwm_locals_dict_table);

const mp_obj_type_t smartcar_pwm_type = {
    { &mp_type_type },
    .name = MP_QSTR_PWM,
    .make_new = pwm_make_new,
    .locals_dict = (mp_obj_dict_t *)&pwm_locals_dict,
};

/**** PWMch type ****/

STATIC const mp_rom_map_elem_t pwmch_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_freq),    MP_ROM_PTR(&pwmch_freq_obj) },
    { MP_ROM_QSTR(MP_QSTR_duty),    MP_ROM_PTR(&pwmch_duty_obj) },
};
STATIC MP_DEFINE_CONST_DICT(pwmch_locals_dict, pwmch_locals_dict_table);

const mp_obj_type_t smartcar_pwmch_type = {
    { &mp_type_type },
    .name = MP_QSTR_PWMch,
    .print  = pwmch_print,
    .locals_dict = (mp_obj_dict_t *)&pwmch_locals_dict,
};
