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
 * Modify:
 *      Ping Liang: 2023 Jun 28
 *          Ported for bare metal Micropython implementation
 */

#include "py/runtime.h"
#include "py/stackctrl.h"
#include "py/objarray.h"
#include "py/objlist.h"
#include "py/gc.h"
#include "mphalport.h"

#include "fsl_pit.h"

#include "Sensor_def.h"

extern const mp_obj_type_t ticker_type;

#define TICKER_PIT_BASEADDR PIT
#define PIT_SOURCE_CLOCK CLOCK_GetPerClkFreq()

#define MAX_TICKER_NUMBER   4
#define MAX_CAPTURE_LIST    8

typedef struct _ticker_obj_t {
    mp_obj_base_t base;

    uint32_t    us_period;
    uint32_t    ticks;      // Count number of interrupts since start()

    mp_obj_base_t *cap_list[MAX_CAPTURE_LIST];
    uint8_t     cap_len;    // Number of capture objects in cap_list
    uint8_t     ticker_id;

    mp_obj_t    callback_hard;
    mp_obj_t    callback_soft;
} ticker_obj_t;

STATIC void ticker_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    ticker_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_print_fun_t func_print;
    mp_printf(print, "Ticker%d: period = %dus, ticks = %d", self->ticker_id, self->us_period, self->ticks);
    if (kind == PRINT_STR)
        return;
    mp_printf(print, "\r\n\thard=");
    func_print = MP_OBJ_TYPE_GET_SLOT(mp_obj_get_type(self->callback_hard), print);
    func_print(print, self->callback_hard, kind);
    mp_printf(print, ", soft=");
    func_print = MP_OBJ_TYPE_GET_SLOT(mp_obj_get_type(self->callback_soft), print);
    func_print(print, self->callback_soft, kind);
    mp_printf(print, "\r\n");

    if (self->cap_len == 0)
        return;
    mp_printf(print, "\tcapture_list:\r\n");
    for (int idx = 0; idx < self->cap_len; idx++) {
        mp_printf(print, "\t[");
        func_print = MP_OBJ_TYPE_GET_SLOT(self->cap_list[idx]->type, print);
    	if (func_print)
    		func_print(print, (mp_obj_t)self->cap_list[idx], kind);
        else
            mp_printf(print, "%q ", self->cap_list[idx]->type->name);
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
/// ticker(id)
STATIC mp_obj_t ticker_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    int ticker_id = mp_obj_get_int(args[0]);
    if (ticker_id > MAX_TICKER_NUMBER)
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("ID must be 0~%d."), MAX_TICKER_NUMBER-1);

    ticker_obj_t *self = mp_obj_malloc(ticker_obj_t, &ticker_type);
    self->cap_len = 0;
    self->us_period = 0;
    self->ticker_id = ticker_id;
    self->callback_hard = mp_const_none;
    self->callback_soft = mp_const_none;
    MP_STATE_VM(ticker_objs[ticker_id]) = self;

    // Find out if there is any ticker object is running.
    // If no one running, then PIT_Init().
    for (ticker_id = 0; ticker_id < MAX_TICKER_NUMBER; ticker_id++) {
        ticker_obj_t *tt = (ticker_obj_t*)MP_STATE_VM(ticker_objs[ticker_id]);
        if (tt && mp_obj_is_type(tt, &ticker_type))
            if (tt->us_period)
                break;
    }
    if (ticker_id == MAX_TICKER_NUMBER) {
        // If there is no running ticker, then initialize the PIT
        pit_config_t pitConfig;
        PIT_GetDefaultConfig(&pitConfig);
        PIT_Init(TICKER_PIT_BASEADDR, &pitConfig);
    }
    return MP_OBJ_FROM_PTR(self);
}

#if defined( __GNUC__ )
extern unsigned int stack_start;
extern unsigned int stack_end;
#else
#error Specify stack allocation variables.
#endif

#ifdef  RT_THREAD
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
#endif  // RT_THREAD

void PIT_IRQHandler(void)
{
    int ticker_id;
    for (ticker_id = 0; ticker_id < MAX_TICKER_NUMBER; ticker_id++) {
        if (PIT_GetStatusFlags(TICKER_PIT_BASEADDR, ticker_id)) {
            PIT_ClearStatusFlags(TICKER_PIT_BASEADDR, ticker_id, 1);
            break;
        }
    }
    if (ticker_id == MAX_TICKER_NUMBER)
        return;

    ticker_obj_t *self = (ticker_obj_t*)MP_STATE_VM(ticker_objs[ticker_id]);
    self->ticks++;

	for (int idx = 0; idx < self->cap_len; idx++) {
        mp_obj_base_t *sensor = self->cap_list[idx];
        const sensor_protocol_t *ppp = MP_OBJ_TYPE_GET_SLOT(sensor->type, protocol);
        ppp->capture(sensor);
    }

	// do a hard call first:
	if (self->callback_hard && (self->callback_hard != mp_const_none)) {
		mp_sched_lock();
        // When executing code within a handler we must lock the GC to prevent
		// any memory allocations.  We must also catch any exceptions.
		gc_lock();
            // fool_stack_check();
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
            // nofool_stack_check();
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
        self->us_period = (uint32_t)(mp_obj_get_float(period_in) * 1000);

        /* Set timer period for channel */
        PIT_SetTimerPeriod(TICKER_PIT_BASEADDR, self->ticker_id, USEC_TO_COUNT(self->us_period, PIT_SOURCE_CLOCK));

        /* Enable timer interrupts for channel */
        PIT_EnableInterrupts(TICKER_PIT_BASEADDR, self->ticker_id, kPIT_TimerInterruptEnable);

        /* Enable at the NVIC */
        EnableIRQ(PIT_IRQn);

        /* Start channel */
        PIT_StartTimer(TICKER_PIT_BASEADDR, self->ticker_id);

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
    PIT_StopTimer(TICKER_PIT_BASEADDR, self->ticker_id);
    PIT_DisableInterrupts(TICKER_PIT_BASEADDR, self->ticker_id, kPIT_TimerInterruptEnable);
    self->us_period = 0;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(ticker_stop_obj, ticker_stop);

#ifdef RT_THREAD
void ticker_init0(void)
{
    int i;
    for (i=0; i < MAX_TICKER_NUMBER; i++) {
        if (ticker[i].pit_deivce)
            ticker_stop(&ticker[i]);
    }
}
#endif  // RT_THREAD

/// \classmethod cap_list(obj1, obj2, ....)
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
        const sensor_protocol_t *ppp = MP_OBJ_TYPE_GET_SLOT(cap_obj->type, protocol);
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

MP_DEFINE_CONST_OBJ_TYPE(
    ticker_type,
    MP_QSTR_Ticker,
    MP_TYPE_FLAG_NONE,
    make_new, ticker_make_new,
    print, ticker_print,
    locals_dict, &ticker_locals_dict
    );

MP_REGISTER_ROOT_POINTER(mp_obj_t ticker_objs[MAX_TICKER_NUMBER]);
