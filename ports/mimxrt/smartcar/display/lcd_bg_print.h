/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Ping Liang
 * Create: 2023, Jun 20
 *
 */

#ifndef _INCLUDE_LCD_BG_PRING_H_
#define _INCLUDE_LCD_BG_PRING_H_

typedef struct _bg_print_task {
    lcd_display_t *lcd;
    uint16_t    pos_x;
    uint16_t    pos_y;
    uint16_t    fg_color;
    uint16_t    bg_color;
    char        *str_buf;
    size_t      str_len;
    const Asc_Font_t *fontp;
    struct _bg_print_task *next;
} bg_print_task_t;

extern void bg_print_sched(bg_print_task_t *task);
extern bool bg_print(bool waitDma);

#endif  // _INCLUDE_LCD_BG_PRING_H_
