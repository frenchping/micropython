/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * Sensor_def.h
 * 
 *  Micropython definitions for sensor interface.
 *  With this interface, other module may interactive with a sensor.
 * 
 */

typedef struct _sensor_protocol_t {
    qstr    name;
    // On error, functions should return MP_STREAM_ERROR and fill in *errcode (values
    // are implementation-dependent, but will be exposed to user, e.g. via exception).
    mp_uint_t (*read)(mp_obj_t obj, void *buf, mp_uint_t size);     // , int *errcode);
    mp_uint_t (*clear)(mp_obj_t obj);
    mp_uint_t (*capture)(mp_obj_t obj);   // Read + Save + Clear
    mp_uint_t (*get)(mp_obj_t obj, void *buf, mp_uint_t size);   // Get the saved value
    mp_uint_t (*ioctl)(mp_obj_t obj, mp_uint_t request, uintptr_t arg, int *errcode);
} sensor_protocol_t;
