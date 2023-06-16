/*
 * PID_py.c
 *
 *  Created on: Dec 30, 2021
 *      Author: nxa19017
 * 
 *  Micropython functions for handling PID calculation
 * 
 */
#include "py/runtime.h"
#include "py/stream.h"
#include "py/mphal.h"

// Virtually, the PID object is a sub-class of an array. Each index represents a channel.
// The index is a non-zero integer. Index=0 means all channels.

extern const mp_obj_type_t pid_var_type;

typedef enum {
    PID_ALGO_INTEGAL = 0,
    PID_ALGO_INCREMENTAL,
} pid_algorithm;

/****************************/
typedef struct {
    float   kp;     // Kp 参数
    float   ki;     // Ki 参数
    float   kd;     // Kd 参数

    pid_algorithm algorithm;

    int error[3];               // 误差列表
    int error_integral;         // 误差积分
    int error_integral_max;     // 误差积分范围 绝对值

    int target;
    int pid_input;              // The input value for retrieve
    int pid_output;             // PID 计算输出

    mp_obj_t    input_obj;      // The object
    mp_obj_t    input_get;      // The function
    mp_obj_t    output_obj;     // The object
    mp_obj_t    output_set;     // The function
} pid_channel_struct;

typedef struct _smartcar_pid_obj_t {
    mp_obj_base_t base;
    int chn_num;        // Number of channels
    pid_channel_struct channel[1];
} smartcar_pid_obj_t;

typedef struct _pid_param_obj_t {
    mp_obj_base_t base;
    smartcar_pid_obj_t *pid_obj;
    int index;
} pid_var_obj_t;

extern const mp_obj_type_t smartcar_pid_type;

/// \classmethod \constructor()
/// Create and return a PID object.
///
/// PID(chn_num)
STATIC mp_obj_t pid_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    int chn_num = mp_obj_get_int(args[0]);
    if (chn_num == 0)
        mp_raise_msg(&mp_type_TypeError, MP_ERROR_TEXT("can not be 0 channels"));

    int obj_size = sizeof(smartcar_pid_obj_t) + (chn_num - 1) * sizeof(pid_channel_struct);
    smartcar_pid_obj_t *self = (smartcar_pid_obj_t*)m_malloc(obj_size);
    memset(self, 0, obj_size);
    self->base.type = &smartcar_pid_type;
    self->chn_num = chn_num;

    return MP_OBJ_FROM_PTR(self);
}

static void pid_std_incremental(pid_channel_struct *chn)
{
    chn->error[2]   = chn->error[1];                // 更新上上次误差
    chn->error[1]   = chn->error[0];                // 更新上次误差
    chn->error[0]   = chn->target - chn->pid_input; // 更新最新误差

    int put_temp;
    put_temp    = chn->kp * (chn->error[0] - chn->error[1]);            // 计算比例控制部分 Kp*(E1-E2)
    put_temp   += chn->ki * chn->error[0];                              // 计算积分控制部分 Ki*E1
    put_temp   += chn->kd * (chn->error[0] - 2*chn->error[1] + chn->error[2]);  // 计算微分控制部分 Kd*(E1-2*E2+E3)

    chn->pid_output += put_temp;
}

static inline float limit(int x, int y)
{
    if(x>y)             return y;
    else if(x<-y)       return -y;
    else                return x;
}

static void pid_std_integral(pid_channel_struct *chn)
{
    chn->error[1]   = chn->error[0];                // 更新上次误差
    chn->error[0]   = chn->target - chn->pid_input; // 更新最新误差

    chn->error_integral += chn->error[0];           // 更新误差积分
    chn->error_integral = limit(chn->error_integral, chn->error_integral_max);  // 误差限幅

    chn->pid_output = chn->kp * chn->error[0];                  // 计算比例控制部分 Kp*E1
    chn->pid_output+= chn->ki * chn->error_integral;            // 计算积分控制部分 Ki*E_integral
    chn->pid_output+= chn->kd * (chn->error[0] - chn->error[1]);// 计算微分控制部分 Kd*(E1-E2)
}

STATIC mp_obj_t pid_go(mp_obj_t self_in)
{
    smartcar_pid_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int ichn;
    pid_channel_struct *chn = &self->channel[0];
    // Step 1: To get all the input first
    for (ichn = 0; ichn < self->chn_num; ichn++, chn++) {
        chn->pid_input = mp_obj_get_int(mp_call_function_0(chn->input_get));
    }
    // Step 2: To calculate all the channels
    chn = &self->channel[0];
    for (ichn = 0; ichn < self->chn_num; ichn++, chn++) {
        if (chn->algorithm == PID_ALGO_INTEGAL)
            pid_std_integral(chn);
        else
            pid_std_incremental(chn);
    }
    // Step 3: To output the result
    chn = &self->channel[0];
    for (ichn = 0; ichn < self->chn_num; ichn++, chn++) {
        mp_obj_t ret = mp_call_function_1(chn->output_set, MP_OBJ_NEW_SMALL_INT(chn->pid_output));
        chn->pid_output = mp_obj_get_int(ret);  // Could be limited by output device. So get this limitation.
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pid_go_obj, pid_go);

// The PID object is a sub-class of an array. Each index represents a channel.
STATIC mp_obj_t pid_subscr(mp_obj_t self_in, mp_obj_t index_in, mp_obj_t value_in)
{
    smartcar_pid_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int index = mp_obj_get_int(index_in);
    if (index && index > self->chn_num)
        mp_raise_msg(&mp_type_TypeError, "wrong index.");
    if (value_in == MP_OBJ_NULL) {
        // delete
        return MP_OBJ_NULL; // op not supported
    } else if (value_in == MP_OBJ_SENTINEL) {
        // load
        pid_var_obj_t *param = m_new_obj(pid_var_obj_t);
        param->base.type = &pid_var_type;
        param->index = index;
        param->pid_obj = self;
        return MP_OBJ_FROM_PTR(param);
    } else {
        // store
        if (mp_obj_is_type(value_in, &pid_var_type)) {
            pid_var_obj_t *param = MP_OBJ_TO_PTR(value_in);
            if (param->index == 0 || index == 0)
                mp_raise_msg(&mp_type_ValueError, "both index can't be 0.");
            // The PIDvar object can only be assign to another PIDvar object.
            pid_channel_struct *from = &param->pid_obj->channel[param->index - 1];
            memcpy(&self->channel[index-1], from, sizeof(pid_channel_struct));
            return mp_const_none;
        }
        mp_raise_msg(&mp_type_TypeError, "wrong assignment.");
    }
}

STATIC const mp_rom_map_elem_t pid_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_go),      MP_ROM_PTR(&pid_go_obj) },
};

STATIC MP_DEFINE_CONST_DICT(pid_locals_dict, pid_locals_dict_table);

const mp_obj_type_t smartcar_pid_type = {
    { &mp_type_type },
    .name = MP_QSTR_PID,
    .make_new = pid_make_new,
    .subscr = pid_subscr,
    .locals_dict = (mp_obj_dict_t *)&pid_locals_dict,
};

/***** Below code handling the virtual array element ***********************/

static bool _IO_obj_get_chk(mp_obj_t from, mp_obj_t **items)
{
    mp_obj_get_array_fixed_n(from, 2, items);
    return mp_obj_is_callable((*items)[1]);
}

STATIC void pid_var_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
	pid_var_obj_t *self = MP_OBJ_TO_PTR(self_in);
    smartcar_pid_obj_t *pid_obj = self->pid_obj;
    pid_channel_struct *chn;
    int index = self->index;

    if (dest[0] == MP_OBJ_NULL) {	// Load
        if (index == 0 || index > pid_obj->chn_num)
            goto do_method;
        chn = &pid_obj->channel[index-1];
        switch (attr) {
            case MP_QSTR_Kp:    dest[0] = mp_obj_new_float(chn->kp);   break;
            case MP_QSTR_Ki:    dest[0] = mp_obj_new_float(chn->ki);   break;
            case MP_QSTR_Kd:    dest[0] = mp_obj_new_float(chn->kd);   break;
            case MP_QSTR_Target:dest[0] = mp_obj_new_int(chn->target);      break;
            case MP_QSTR_Algo:  dest[0] = mp_obj_new_int(chn->algorithm);   break;
            case MP_QSTR_Input: dest[0] = mp_obj_new_int(chn->pid_input);   break;
            case MP_QSTR_Output:dest[0] = mp_obj_new_int(chn->pid_output);  break;
            case MP_QSTR_Errmax:dest[0] = mp_obj_new_int(chn->error_integral_max);break;
            default:
    do_method:
                dest[0] = mp_obj_dict_get((mp_obj_t)pid_var_type.locals_dict, MP_ROM_QSTR(attr));
                dest[1] = self_in;
                break;
        }
    }
    else {
        if (dest[1] != MP_OBJ_NULL) {   // Store
            mp_obj_t *IO_items;
            if (index == 0) {
                // Index = 0, means to assign all the channels to the same value
                chn = &pid_obj->channel[0];
                for (index = 0; index < pid_obj->chn_num; index++, chn++) {
                    switch (attr) {
                        case MP_QSTR_Kp:    chn->kp = mp_obj_get_float(dest[1]);   break;
                        case MP_QSTR_Ki:    chn->ki = mp_obj_get_float(dest[1]);   break;
                        case MP_QSTR_Kd:    chn->kd = mp_obj_get_float(dest[1]);   break;
                        case MP_QSTR_Target:chn->target = mp_obj_get_int(dest[1]); break;
                        case MP_QSTR_Algo:  chn->algorithm = mp_obj_get_int(dest[1]); break;
                        case MP_QSTR_Errmax:chn->error_integral_max = mp_obj_get_int(dest[1]); break;
                        case MP_QSTR_Get:
                            if (!_IO_obj_get_chk(dest[1], &IO_items)) {
                                mp_raise_msg(&mp_type_TypeError, "param is not a function.");
                                return;     // Not a function
                            }
                            chn->input_obj = IO_items[0];
                            chn->input_get = IO_items[1];
                            break;
                        case MP_QSTR_Set:
                            if (!_IO_obj_get_chk(dest[1], &IO_items)) {
                                mp_raise_msg(&mp_type_TypeError, "param is not a function.");
                                return;     // Not a function
                            }
                            chn->output_obj = IO_items[0];
                            chn->output_set = IO_items[1];
                            break;
                        default:
                            return; // Not a member
                    }
                }
            }
            else {
                if (index > pid_obj->chn_num)
                    mp_raise_ValueError("index is out of range.");
                chn = &pid_obj->channel[index-1];
                switch (attr) {
                    case MP_QSTR_Kp:    chn->kp = mp_obj_get_float(dest[1]);   break;
                    case MP_QSTR_Ki:    chn->ki = mp_obj_get_float(dest[1]);   break;
                    case MP_QSTR_Kd:    chn->kd = mp_obj_get_float(dest[1]);   break;
                    case MP_QSTR_Target:chn->target = mp_obj_get_int(dest[1]); break;
                    case MP_QSTR_Algo:  chn->algorithm = mp_obj_get_int(dest[1]); break;
                    case MP_QSTR_Errmax:chn->error_integral_max = mp_obj_get_int(dest[1]); break;
                    case MP_QSTR_Get: 
                        if (!_IO_obj_get_chk(dest[1], &IO_items)) {
                            mp_raise_msg(&mp_type_TypeError, "param is not a function.");
                            return;     // Not a function
                        }
                        chn->input_obj = IO_items[0];
                        chn->input_get = IO_items[1];
                        break;
                    case MP_QSTR_Set:
                        if (!_IO_obj_get_chk(dest[1], &IO_items)) {
                            mp_raise_msg(&mp_type_TypeError, "param is not a function.");
                            return;     // Not a function
                        }
                        chn->output_obj = IO_items[0];
                        chn->output_set = IO_items[1];
                        break;
                    default:
                        return; // Not a member
                }
            }
            dest[0] = MP_OBJ_NULL; // indicate success
            return;
        }
    }
}

STATIC mp_obj_t pid_var_clear(mp_obj_t self_in)
{
    pid_var_obj_t *self = MP_OBJ_TO_PTR(self_in);
    pid_channel_struct *chn = &self->pid_obj->channel[0];
    int index = self->index;
    
    if (index == 0) {
        for (index = 0; index < self->pid_obj->chn_num; index++, chn++) {
            chn->error[0] = chn->error[1] = chn->error[2] = 0;
            chn->pid_output = 0;
            chn->error_integral = 0;
        }
    }
    else {
        chn->error[0] = chn->error[1] = chn->error[2] = 0;
        chn->pid_output = 0;
        chn->error_integral = 0;
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pid_var_clear_obj, pid_var_clear);

STATIC void pid_var_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    pid_var_obj_t *self = MP_OBJ_TO_PTR(self_in);
    pid_channel_struct *chn = &self->pid_obj->channel[self->index - 1];

    mp_printf(print, "   PID channel (%d) parameters:\r\n", self->index);
    mp_printf(print, "\tTarget = %d;\tInput = %d;\tOutput = %d\r\n", chn->target, chn->pid_input, chn->pid_output);
    mp_printf(print, "\tKp = %6.3f;\tKi = %6.3f;\tKd = %6.3f\r\n", chn->kp, chn->ki, chn->kd);
    mp_printf(print, "\tErr0 = %d;\tErr1 = %d;\tErr2 = %d\r\n", chn->error[0], chn->error[1], chn->error[2]);
    mp_printf(print, "\terror_integral = %d;\terror_integral_max = %d", chn->error_integral, chn->error_integral_max);
}

STATIC const mp_rom_map_elem_t pid_var_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_clear),      MP_ROM_PTR(&pid_var_clear_obj) },
};
STATIC MP_DEFINE_CONST_DICT(pid_var_locals_dict, pid_var_locals_dict_table);

const mp_obj_type_t pid_var_type = {
    { &mp_type_type },
    .name = MP_QSTR_PIDvar,
    .print = pid_var_print,
    .attr = pid_var_attr,
    .locals_dict = (mp_obj_dict_t *)&pid_var_locals_dict,
};
