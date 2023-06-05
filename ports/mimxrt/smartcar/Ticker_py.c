/*
 * ticker_py.c
 *
 *  Created on: Dec 17, 2021
 *      Author: nxa19017
 * 
 *  Micropython functions for manipulate multiple ADC conversion
 *  
 *  With this class, multiple ADC channels can be converted in sequence and periodically
 * 
 */

#include "py/runtime.h"
#include "py/stackctrl.h"
#include "py/objarray.h"
#include "py/objlist.h"
#include "py/gc.h"
#include "mphalport.h"


#include "fsl_pit.h"
#include "drv_pit.h"

#include "Sensor_def.h"

extern const mp_obj_type_t smartcar_ticker_type;

#define MAX_TICKER_NUMBER   2
#define MAX_CAPTURE_LIST    8

typedef struct _ticker_obj_t {
    mp_obj_base_t base;
    rt_device_t pit_deivce;

    uint32_t    us_period;

    mp_obj_base_t *cap_list[MAX_CAPTURE_LIST];
    int     cap_len;    // Number of capture objects in cap_list
    uint32_t    ticks;  // Count number of interrupts since start()

    mp_obj_t    callback_hard;
    mp_obj_t    callback_soft;
} ticker_obj_t;
ticker_obj_t ticker[MAX_TICKER_NUMBER];

#define MP_PIT_DEV_NAME "pit"

STATIC void ticker_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    ticker_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_obj_type_t *type;
    mp_printf(print, "Ticker%d: period = %dus, ticks = %d", self - ticker + 1, self->us_period, self->ticks);
    if (kind == PRINT_STR)
        return;
    mp_printf(print, "\r\n\thard=", self - ticker);
    type = mp_obj_get_type(self->callback_hard);
    type->print(print, self->callback_hard, kind);
    mp_printf(print, ", soft=");
    type = mp_obj_get_type(self->callback_soft);
    type->print(print, self->callback_soft, kind);
    mp_printf(print, "\r\n");

    if (self->cap_len == 0)
        return;
    mp_printf(print, "\tcapture_list:\r\n");
    for (int idx = 0; idx < self->cap_len; idx++) {
        mp_printf(print, "\t[");
        type = self->cap_list[idx]->type;
    	if (type->print)
    		type->print(print, (mp_obj_t)self->cap_list[idx], kind);
        else
            mp_printf(print, "%q ", type->name);
    	mp_printf(print, "]");
        if (idx + 1 != self->cap_len)
            mp_printf(print, ",\r\n");
    }
}

/// \classmethod \constructor()
/// Create and return a Ticker object.
///
/// Construct an object with ID
///
/// Timed_ADC(id, group)
STATIC mp_obj_t ticker_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    int ticker_id = mp_obj_get_int(args[0]);
    if (ticker_id < 1 || ticker_id > MAX_TICKER_NUMBER)
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("ID must be 1~%d."), MAX_TICKER_NUMBER);

    ticker_obj_t *self = &ticker[ticker_id - 1];
    self->base.type = &smartcar_ticker_type;
    self->pit_deivce = rt_device_find(MP_PIT_DEV_NAME);
    if (self->pit_deivce == RT_NULL)
        mp_raise_ValueError("PIT don't exist");
    self->cap_len = 0;
    self->callback_hard = mp_const_none;
    self->callback_soft = mp_const_none;

    return MP_OBJ_FROM_PTR(self);
}

#if defined( __GNUC__ )
extern unsigned int stack_start;
extern unsigned int stack_end;
#else
#error Specify stack allocation variables.
#endif

// static CONTROL_Type CONTROL;
static char *psp_top;
static size_t psp_limit;

void fool_stack_check(void)
{
    register uint32_t ctrl asm ("r3");  // Used to know what stack (msp or psp) is using
    asm ("mrs %0, CONTROL" : "=r" (ctrl));
    if (ctrl & CONTROL_SPSEL_Msk)
        return;     // This is the process stack. No fool
    psp_top = MP_STATE_THREAD(stack_top);
    psp_limit = MP_STATE_THREAD(stack_limit);
    MP_STATE_THREAD(stack_top) = (char *)&stack_end;            // mp_stack_set_top(&stack_end);
    MP_STATE_THREAD(stack_limit) = &stack_start - &stack_end;   // mp_stack_set_limit(&stack_start - &stack_end);
}
void nofool_stack_check(void)
{
    register uint32_t ctrl asm ("r3");  // Used to know what stack (msp or psp) is using
    asm ("mrs %0, CONTROL" : "=r" (ctrl));
    if (ctrl & CONTROL_SPSEL_Msk)
        return;     // This is the process stack. No fool
    
    MP_STATE_THREAD(stack_top) = psp_top;       // mp_stack_set_top(psp_top);
    MP_STATE_THREAD(stack_limit) = psp_limit;   // mp_stack_set_limit(psp_limit);
}

static void pit_isr(int chn)
{
    ticker_obj_t *self = &ticker[chn - 1];
    self->ticks++;
	for (int idx = 0; idx < self->cap_len; idx++) {
        mp_obj_base_t *sensor = self->cap_list[idx];
        const sensor_protocol_t *ppp = sensor->type->protocol;
        ppp->capture(sensor);
    }

	// do a hard call first:
	if (self->callback_hard && (self->callback_hard != mp_const_none)) {
		mp_sched_lock();
        // When executing code within a handler we must lock the GC to prevent
		// any memory allocations.  We must also catch any exceptions.
		gc_lock();
            fool_stack_check();
		nlr_buf_t nlr;
		if(nlr_push(&nlr) == 0){
			mp_call_function_1(self->callback_hard, self);
			nlr_pop();
		}else{
			// Uncaught exception; disable the callback so it doesn't run again.
			self->callback_hard = mp_const_none;
			//__HAL_TIM_DISABLE_IT(&tim->tim, irq_mask);
			mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
		}
            nofool_stack_check();
		gc_unlock();
		mp_sched_unlock();
	}
    // Then do a soft call:
    if (self->callback_soft && (self->callback_soft != mp_const_none)) {
        mp_sched_schedule(self->callback_soft, MP_OBJ_FROM_PTR(self));
    }
}

/// \classmethod start(period)
STATIC mp_obj_t ticker_start(mp_obj_t self_in, mp_obj_t period_in)
{
    ticker_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (self->cap_len != 0 ||
            (self->callback_soft && (self->callback_soft != mp_const_none)) ||
            (self->callback_hard && (self->callback_hard != mp_const_none)) ) {
        self->us_period = mp_obj_get_float(period_in) * 1000;
        pit_params_t pit_param = {
            .chn    = (self - ticker) + 1,
            .us_period = self->us_period,
            .callback = pit_isr,
        };
        rt_device_control(self->pit_deivce, RT_DEVICE_CTRL_PIT_START, &pit_param);
        rt_device_control(self->pit_deivce, RT_DEVICE_CTRL_PIT_CALLBACK, &pit_param);
        self->ticks = 0;
    }
    else
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("capture_list is empty and no-callback"));

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(ticker_start_obj, ticker_start);

/// \classmethod stop()
STATIC mp_obj_t ticker_stop(mp_obj_t self_in)
{
    ticker_obj_t *self = MP_OBJ_TO_PTR(self_in);
    pit_params_t pit_param = {
        .chn    = (self - ticker) + 1,
    };
    rt_device_control(self->pit_deivce, RT_DEVICE_CTRL_PIT_STOP, &pit_param);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(ticker_stop_obj, ticker_stop);

void ticker_init0(void)
{
    int i;
    for (i=0; i < MAX_TICKER_NUMBER; i++) {
        if (ticker[i].pit_deivce)
            ticker_stop(&ticker[i]);
    }
}

/// \classmethod callback(soft, [hard])
STATIC mp_obj_t ticker_cap_list(size_t n_args, const mp_obj_t *args)
{
    ticker_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    self->cap_len = 0;
    int idx;
    for (idx = 1; idx < MAX_CAPTURE_LIST+1; idx++) {
        if (n_args == idx)
            break;
        mp_obj_base_t *cap_obj = (mp_obj_base_t *)args[idx];
        if (!mp_obj_is_obj(cap_obj))
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("arg%d is not an object."), idx);;
        const sensor_protocol_t *ppp = cap_obj->type->protocol;
        if (ppp->name != MP_QSTR_Sensor)
            mp_raise_msg_varg(&mp_type_TypeError, MP_ERROR_TEXT("arg%d(%q) is supported."), idx, cap_obj->type->name);;
        self->cap_list[idx-1] = cap_obj;
    }
    self->cap_len = idx-1;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(ticker_cap_list_obj, 1, MAX_CAPTURE_LIST+1, ticker_cap_list);

/// \classmethod ticks()
/// Return the current ticks
STATIC mp_obj_t ticker_ticks(mp_obj_t self_in)
{
    ticker_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(self->ticks);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(ticker_ticks_obj, ticker_ticks);

/// \classmethod callback(soft, [hard])
STATIC mp_obj_t ticker_callback(size_t n_args, const mp_obj_t *args)
{
    ticker_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (n_args >= 2) {
        self->callback_soft = MP_OBJ_TO_PTR(args[1]);
    }
    if (n_args == 3) {
        self->callback_hard = MP_OBJ_TO_PTR(args[2]);
    }
    else
        self->callback_hard = mp_const_none;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(ticker_callback_obj, 2, 3, ticker_callback);

STATIC const mp_rom_map_elem_t ticker_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_capture_list),MP_ROM_PTR(&ticker_cap_list_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),   MP_ROM_PTR(&ticker_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),    MP_ROM_PTR(&ticker_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_callback),MP_ROM_PTR(&ticker_callback_obj) },
    { MP_ROM_QSTR(MP_QSTR_ticks),   MP_ROM_PTR(&ticker_ticks_obj) },
};

STATIC MP_DEFINE_CONST_DICT(ticker_locals_dict, ticker_locals_dict_table);

const mp_obj_type_t smartcar_ticker_type = {
    { &mp_type_type },
    .name = MP_QSTR_Ticker,
    .make_new = ticker_make_new,
    // .attr   = ticker_attr,
    .print = ticker_print,
    .locals_dict = (mp_obj_dict_t *)&ticker_locals_dict,
};
