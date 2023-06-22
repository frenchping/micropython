/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Ping Liang
 * Create: 2023, Jun 20
 *
 */

/*
 * Print characters on LCD at background.
 */

#include <stdio.h>
#include "py/runtime.h"

#include "pin.h"

#include "lcd_fonts.h"
#include "lcd_spi_drv.h"
#include "lcd_bg_print.h"

#define PRINT_CHAR_NUM      1       // Number characters output in each schedule slice

#define EDMA_PRINT_BUF_SIZE ((16*32*2)*PRINT_CHAR_NUM)
AT_NONCACHEABLE_SECTION_ALIGN(uint16_t print_buf[EDMA_PRINT_BUF_SIZE], 16);

// Output PRINT_CHAR_NUM characters from the task
// Return 'true' if the task is finished.
bool bg_print_slice(bg_print_task_t *task)
{
    if (!spi_lcd_dma_done())
        return false;

    const Asc_Font_t *fontp = task->fontp;
    if ((task->pos_y + fontp->Height) > task->lcd->Height)
        return true;      // Exceed the screen height

    int font_buf_size = fontp->Width * fontp->Height * sizeof(uint16_t);
    // uint16_t *BMP_buf = (uint16_t*)m_malloc(font_buf_size);

    bool ret = false;
    for (int ichr = 0; ichr < PRINT_CHAR_NUM; ichr++) {
        if (task->str_len == 0 || (task->pos_x + fontp->Width) > task->lcd->Width) {
            ret = true;
            break;
        }

        task->str_len--;
        int chr = *task->str_buf++ - ' ';
        uint16_t *bmp_buf = print_buf;
        const uint8_t *cfont = fontp->Fonts + chr * fontp->Bytes; // Pointer to the character font
        for (int fonty = 0; fonty < fontp->Height; fonty++) {
            uint16_t bmp = *cfont++;
            if (fontp->BytesOnWidth == 2)
                bmp |= *cfont++ << 8;
            uint16_t mask = 1;
            for (int fontx = 0; fontx < fontp->Width; fontx++) {
                if (bmp & mask)
                    *bmp_buf = task->fg_color;
                else
                    *bmp_buf = task->bg_color;
                bmp_buf++;
                mask <<= 1;
            }
        }
        Rect_t rect = { task->pos_x, task->pos_y, fontp->Width, fontp->Height };
        spi_lcd_set_window(task->lcd, &rect);
        spi_lcd_dma_fill(task->lcd, print_buf, font_buf_size / 2, 0);
        task->pos_x += fontp->Width;
    }
    // m_free(BMP_buf);
    return ret;
}

// Return false if nothing to output.
bool bg_print(bool waitDma)
{
    bg_print_task_t *task = MP_STATE_VM(bg_task_head);
	if (task == NULL)
        return false;

    if (bg_print_slice(task)) {
        MP_STATE_VM(bg_task_head) = task->next;
        if (MP_STATE_VM(bg_task_head) == NULL)
            MP_STATE_VM(bg_task_tail) = NULL;
    }

    if (waitDma)
        while (!spi_lcd_dma_done())
            __NOP();
    return true;
}

void bg_print_sched(bg_print_task_t *task)
{
    if (MP_STATE_VM(bg_task_tail) == NULL) {
        MP_STATE_VM(bg_task_head) = task;
    } else {
        ((bg_print_task_t*)MP_STATE_VM(bg_task_tail))->next = task;
    }
    MP_STATE_VM(bg_task_tail) = task;
}

MP_REGISTER_ROOT_POINTER(void *bg_task_head);
MP_REGISTER_ROOT_POINTER(void *bg_task_tail);
