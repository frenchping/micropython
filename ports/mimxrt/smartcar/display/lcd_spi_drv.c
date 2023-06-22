/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Philipp Ebensberger
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * Author: Ping Liang
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "py/obj.h"
#include "pin.h"

#include "mphalport.h"

#include "fsl_lpspi.h"
#include "lcd_spi_drv.h"



void lcd180_init(lcd_display_t *lcd);
void lcd180_set_mode(lcd_display_t *lcd, Display_Orientation mode);
void lcd350_init(lcd_display_t *lcd);
void lcd350_set_mode(lcd_display_t *lcd, Display_Orientation mode);
void lcd154_init(lcd_display_t *lcd);
void lcd154_set_mode(lcd_display_t *lcd, Display_Orientation mode);
void lcd200_init(lcd_display_t *lcd);
void lcd200_set_mode(lcd_display_t *lcd, Display_Orientation mode);

const lcd_hw_attribute_t lcd_drivers[SPI_LCD_NUM_TYPE] = {
    [SPI_LCD_200].lcd_init		= lcd200_init,
    [SPI_LCD_200].lcd_set_mode	= lcd200_set_mode,
    [SPI_LCD_200].lcd_width     = LCD200_WIDTH,
    [SPI_LCD_200].lcd_height    = LCD200_HEIGHT,
    [SPI_LCD_200].color_depth   = LCD200_COLORS,
    /*
    [SPI_LCD_180].lcd_init		= lcd180_init,
    [SPI_LCD_180].lcd_set_mode	= lcd180_set_mode,
    [SPI_LCD_180].lcd_width     = LCD180_WIDTH,
    [SPI_LCD_180].lcd_height    = LCD180_HEIGHT,
    [SPI_LCD_180].color_depth   = LCD180_COLORS,

    [SPI_LCD_350].lcd_init		= lcd350_init,
    [SPI_LCD_350].lcd_set_mode	= lcd350_set_mode,
    [SPI_LCD_350].lcd_width     = LCD350_WIDTH,
    [SPI_LCD_350].lcd_height    = LCD350_HEIGHT,
    [SPI_LCD_350].color_depth   = LCD350_COLORS,

    [SPI_LCD_154].lcd_init		= lcd154_init,
    [SPI_LCD_154].lcd_set_mode	= lcd154_set_mode,
    [SPI_LCD_154].lcd_width     = LCD154_WIDTH,
    [SPI_LCD_154].lcd_height    = LCD154_HEIGHT,
    [SPI_LCD_154].color_depth   = LCD154_COLORS,
    */
};

#define LPSPI_CMD8b_TCR(tcr)		tcr = ((tcr) & ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK | LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_PCS_MASK | LPSPI_TCR_FRAMESZ_MASK | LPSPI_TCR_PRESCALE_MASK)) | \
									(LPSPI_TCR_RXMSK(1) | LPSPI_TCR_PCS(0) | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_PRESCALE(1))
#define LPSPI_DATA8b_TCR(tcr)		tcr = ((tcr) & ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK | LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_PCS_MASK | LPSPI_TCR_FRAMESZ_MASK | LPSPI_TCR_PRESCALE_MASK)) | \
									(LPSPI_TCR_RXMSK(1) | LPSPI_TCR_PCS(0) | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_PRESCALE(1))
#define LPSPI_16Bit_TCR(tcr)		((tcr) & ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK | LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_PCS_MASK | LPSPI_TCR_FRAMESZ_MASK | LPSPI_TCR_PRESCALE_MASK)) | \
									(LPSPI_TCR_RXMSK(1) | LPSPI_TCR_PCS(0) | LPSPI_TCR_FRAMESZ(15))
#define SET_DC_CMD          mp_hal_pin_low(lcd->dc_pin)
#define SET_DC_DATA         mp_hal_pin_high(lcd->dc_pin)

#define Send_Cmd(cmd)	    spi_lcd_write_cmd(lcd, cmd, sizeof(cmd)-1)

/*******************************************************************************
        Send command with SPI registers directly
*******************************************************************************/
static inline void LPSPI_Write_Data(LPSPI_Type *spi, uint16_t val)
{
	while (LPSPI_GetTxFifoCount(spi) == 16)
		;	// Wait FIFO is not full
	spi->TDR = val;
}

// Wait All transfers have completed
static inline void LPSPI_Wait_Idle(LPSPI_Type *spi)
{
	while (spi->SR & kLPSPI_ModuleBusyFlag) {
		__NOP();
		__NOP();
	}
}

#define Wait_Transfer_Complete  { \
    for (int timeout=500; timeout>0; timeout--) \
        if (spi->SR & kLPSPI_TransferCompleteFlag)  \
            break;  \
}

/********************************************************
 * Write LCD register
********************************************************/
void spi_lcd_write_reg(lcd_display_t *lcd, uint8_t Reg)
{
    LPSPI_Type *spi = lcd->spi_base;

    LPSPI_Wait_Idle(spi);
    SET_DC_CMD;
    LPSPI_CMD8b_TCR(spi->TCR);

    LPSPI_ClearStatusFlags(spi, kLPSPI_AllStatusFlag);
    LPSPI_Write_Data(spi, Reg);

    Wait_Transfer_Complete;
    LPSPI_ClearStatusFlags(spi, kLPSPI_AllStatusFlag);
    SET_DC_DATA;
}

/********************************************************
 * Write LCD register followed by parameters.
********************************************************/
void spi_lcd_write_cmd(lcd_display_t *lcd, const uint8_t *cmd, int len)
{
    LPSPI_Type *spi = lcd->spi_base;
    spi_lcd_write_reg(lcd, *cmd++);

    LPSPI_DATA8b_TCR(spi->TCR);
    SET_DC_DATA;
    if (len) {
        for (; len; len--)
            LPSPI_Write_Data(spi, *cmd++);
        Wait_Transfer_Complete;
    }
}

/********************************************************
 * Write 16-bit of data.
********************************************************/
void spi_lcd_write_data(lcd_display_t *lcd, uint16_t Data)
{
    LPSPI_Type *spi = lcd->spi_base;
	LPSPI_Wait_Idle(spi);

	spi->TCR = LPSPI_16Bit_TCR(spi->TCR);
	LPSPI_Write_Data(spi, Data);
}


void lcd200_init(lcd_display_t *lcd)
{
    static const uint8_t init_seq[] = {
        1, 0x3A, 0x05,                  // 3A cmd, 1 byte data
        5, 0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33,      // B2 cmd, 5 bytes data
        1, 0xB7, 0x35,                  // B7 cmd, 1 byte data
        1, 0xBB, 0x29,                  // 32 Vcom=1.35V
        1, 0xC2, 0x01,                  // C2 cmd, 1 byte data
        1, 0xC3, 0x19,                  //  GVDD=4.8V 
        1, 0xC4, 0x20,                  //  VDV, 0x20:0v
        1, 0xC5, 0x1A,                  //  VCOM Offset Set
        1, 0xC6, 0x1F,                  //  0x0F:60Hz
        2, 0xD0, 0xA4, 0xA1,            // D0 cmd, 2 bytes data
        14,0xE0, 0xD0, 0x08, 0x0E, 0x09,
                0x09, 0x05, 0x31, 0x33,
                0x48, 0x17, 0x14, 0x15,
                0x31, 0x34,             // E0 cmd, 14 bytes data
        14,0xE1, 0xD0, 0x08, 0x0E, 0x09,
                0x09, 0x15, 0x31, 0x33,
                0x48, 0x17, 0x14, 0x15,
                0x31, 0x34,             // E1 cmd, 14 bytes data

        0, 0x21,
        0, 0
    };

    lcd->Screen_Width = LCD200_WIDTH;
    lcd->Screen_Height= LCD200_HEIGHT;
    lcd->Color_Depth  = LCD200_COLORS;
    // LPSPI_Type *spi = lcd->spi_base;
    // Enable the maximum speed
    // spi->CCR = 0;		// This is a shortcut instead of SDK functions

    //sleep out
    spi_lcd_write_reg(lcd, 0x11);
    mp_hal_delay_ms(120);           // Wait 120ms

    const uint8_t *cmdp = init_seq;
    while (cmdp[0] || cmdp[1]) {
        spi_lcd_write_cmd(lcd, cmdp+1, cmdp[0]);
        cmdp += cmdp[0] + 2;
    }

	// lcd200_set_mode(lcd, DEFAULT_ORIENTATION);
    mp_hal_delay_ms(100);           // Wait 100ms

	//Turn on the LCD display
	spi_lcd_write_reg(lcd, 0x29);
}

/********************************************************
 *  Orientation_Control[] is used for ST7735 & ST7789
********************************************************/
static const uint8_t Orientation_Control[ORIENT_MAX] = {
    (1<<7 | 1<<6 | 0<<5),	// ORIENT_0
    (1<<7 | 0<<6 | 1<<5),	// ORIENT_90
    (0<<7 | 0<<6 | 0<<5),	// ORIENT_180
    (0<<7 | 1<<6 | 1<<5),	// ORIENT_270

    (1<<7 | 1<<6 | 0<<5 | 1<<4),	// ORIENT_MIRROR_0
    (1<<7 | 0<<6 | 1<<5 | 1<<2),	// ORIENT_MIRROR_90
    (0<<7 | 0<<6 | 0<<5 | 1<<4),	// ORIENT_MIRROR_180
    (0<<7 | 1<<6 | 1<<5 | 1<<2),	// ORIENT_MIRROR_270
};

void lcd200_set_mode(lcd_display_t *lcd, Display_Orientation mode)
{
    uint8_t init_36[2];
    init_36[0] = 0x36;
    init_36[1] = Orientation_Control[mode];
    Send_Cmd(init_36);
}

/* To check if a pre-defined panel is used. */
#define PRE_DEFINED_SCREEN(lcd) ((lcd->lcd_type >= 0) && (lcd->lcd_type < SPI_LCD_NUM_TYPE))

/********************************************************
 * Called from Python layer
********************************************************/
void spi_lcd_set_mode(lcd_display_t *lcd, Display_Orientation mode)
{
    lcd->Origin_shift_x = 0;
    lcd->Origin_shift_y = 0;

    //Get the screen scan direction
    if (mode == ORIENT_0 || mode == ORIENT_MIRROR_0 ||
        mode == ORIENT_180 || mode == ORIENT_MIRROR_180) {
        lcd->Width	= lcd->Screen_Height ;
        lcd->Height = lcd->Screen_Width ;
    }
    else {
        lcd->Width	= lcd->Screen_Width ;
        lcd->Height = lcd->Screen_Height ;
    }

    // If a user panel is used, call user defined function
    if (!PRE_DEFINED_SCREEN(lcd) || lcd->user_ops.set_mode)
        lcd->user_ops.set_mode(lcd->user_ops.user_obj, mode);     // Call user defined function
    else
	    lcd_drivers[lcd->lcd_type].lcd_set_mode(lcd, mode);
    lcd->Orientation = mode;
}

void spi_lcd_set_window_direct(lcd_display_t *lcd, Rect_t *rect)
{
	//set the X coordinates
	uint16_t	left = rect->x0 + lcd->Origin_shift_x;
	uint16_t	right = rect->x0 + rect->width + lcd->Origin_shift_x -1;
	uint8_t cmdX[5] = { 0x2A, left >> 8, left & 0xff, right >> 8, right & 0xff };
	Send_Cmd(cmdX);

	//set the Y coordinates
	uint16_t	top = rect->y0 + lcd->Origin_shift_y;
	uint16_t	bottom = rect->y0 + rect->height + lcd->Origin_shift_y - 1;
	uint8_t cmdY[5] = { 0x2B, top >> 8, top & 0xff, bottom >> 8, bottom & 0xff };
	Send_Cmd(cmdY);
	spi_lcd_write_reg(lcd, 0x2C);
// #ifndef  _SPI_DRIVE_
//     config_spi_pins(lcd);
// #endif
}

void spi_lcd_set_window(lcd_display_t *lcd, Rect_t *rect)
{
    // If a user panel is used, call user defined function
    if (!PRE_DEFINED_SCREEN(lcd) || lcd->user_ops.set_window)
        lcd->user_ops.set_window(lcd->user_ops.user_obj, rect);     // Call user defined function
    else
        spi_lcd_set_window_direct(lcd, rect);
}

void spi_lcd_fill_color(lcd_display_t *lcd, uint16_t color, uint32_t length)
{
    LPSPI_Type *spi = lcd->spi_base;
    LPSPI_Wait_Idle(spi);

    spi->TCR = LPSPI_16Bit_TCR(spi->TCR) | LPSPI_TCR_CONT_MASK;
    for (; length; length--) {
        LPSPI_Write_Data(spi, color);
    }
    while (spi->FSR & LPSPI_FSR_TXCOUNT_MASK)
        __NOP();	// Wait FIFO empty
    LPSPI_DATA8b_TCR(spi->TCR);				// Write a new TCR to start a new frame ==> deassert PCS
}

#include "fsl_lpspi_edma.h"
#include "fsl_dmamux.h"
volatile bool isDmaTransferOnGoing;

#define EDMA_BUFFER_SIZE    (512*8)
AT_NONCACHEABLE_SECTION_ALIGN(uint16_t edma_buf[EDMA_BUFFER_SIZE], 16);

void spi_lcd_dma_callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    LPSPI_Type *spi_base = (LPSPI_Type*)param;
    while (spi_base->FSR & LPSPI_FSR_TXCOUNT_MASK)
        __NOP();	// Wait FIFO empty

    LPSPI_EnableDMA(spi_base, false);
    LPSPI_DATA8b_TCR(spi_base->TCR);            // Write a new TCR to start a new frame ==> deassert PCS

    isDmaTransferOnGoing = false;
}

bool spi_lcd_dma_done()
{
    return !isDmaTransferOnGoing;
}

static edma_handle_t           g_EDMA_handle;
void spi_lcd_dma_init(lcd_display_t *lcd, uint32_t dma_chn)
{
    dma_request_source_t    dma_src[] = DMA_REQ_SRC_TX;
    DMAMUX_Init(DMAMUX);
    DMAMUX_SetSource(DMAMUX, dma_chn, dma_src[lcd->lpspi_id]);
    DMAMUX_EnableChannel(DMAMUX, dma_chn);

    edma_transfer_config_t  transferConfig;
    DMA0->CERR = DMA_CERR_CERR_MASK | DMA_CERR_CAEI_MASK;

    EDMA_CreateHandle(&g_EDMA_handle, DMA0, dma_chn);
    EDMA_SetCallback(&g_EDMA_handle, spi_lcd_dma_callback, lcd->spi_base);
    EDMA_PrepareTransfer(&transferConfig, 
                                (void *)0x12345678, // void *srcAddr,
                                sizeof(uint16_t),   // uint32_t srcWidth
                                (void *)(&lcd->spi_base->TDR),   // void *destAddr
                                sizeof(uint16_t),   // uint32_t destWidth
                                sizeof(uint16_t),   // uint32_t bytesEachRequest
                                0xABC * sizeof(uint16_t),   // uint32_t transferBytes
                                kEDMA_MemoryToPeripheral);

    EDMA_SubmitTransfer(&g_EDMA_handle, &transferConfig);
    EDMA_SetModulo(DMA0, dma_chn, kEDMA_ModuloDisable, kEDMA_ModuloDisable);
}

void spi_lcd_dma_fill(lcd_display_t *lcd, uint16_t *buffer, uint32_t length, uint32_t dma_chn)
{
#if 0   // This part is moved to spi_lcd_dma_init() to save time for initialization in everytime.
    dma_request_source_t    dma_src[] = DMA_REQ_SRC_TX;
    edma_transfer_config_t  transferConfig;
    edma_config_t           edma_config;
    DMAMUX_Init(DMAMUX);

    DMAMUX_SetSource(DMAMUX, dma_chn, dma_src[lcd->lpspi_id]);
    DMAMUX_EnableChannel(DMAMUX, dma_chn);

    EDMA_GetDefaultConfig(&edma_config);
    EDMA_Deinit(DMA0);
    EDMA_Init(DMA0, &edma_config);
    DMA0->CERR = DMA_CERR_CERR_MASK | DMA_CERR_CAEI_MASK;

    EDMA_CreateHandle(&g_EDMA_handle, DMA0, dma_chn);

    EDMA_SetCallback(&g_EDMA_handle, spi_lcd_dma_callback, lcd->spi_base);
    EDMA_PrepareTransfer(&transferConfig, 
                                (void *)buffer,     // void *srcAddr,
                                sizeof(uint16_t),   // uint32_t srcWidth
                                (void *)(&lcd->spi_base->TDR),   // void *destAddr
                                sizeof(uint16_t),   // uint32_t destWidth
                                sizeof(uint16_t),   // uint32_t bytesEachRequest
                                length * sizeof(uint16_t),  // uint32_t transferBytes
                                kEDMA_MemoryToPeripheral);

    EDMA_SubmitTransfer(&g_EDMA_handle, &transferConfig);
    EDMA_SetModulo(DMA0, dma_chn, kEDMA_ModuloDisable, kEDMA_ModuloDisable);
#else
    edma_tcd_t *tcd = (edma_tcd_t *)&g_EDMA_handle.base->TCD[g_EDMA_handle.channel];
    tcd->SADDR = (uint32_t)buffer;
    tcd->CITER = length;
    tcd->BITER = length;
#endif
    lcd->spi_base->TCR = LPSPI_16Bit_TCR(lcd->spi_base->TCR) | LPSPI_TCR_CONT_MASK;    
    LPSPI_EnableDMA(lcd->spi_base, true);
    EDMA_StartTransfer(&g_EDMA_handle);

    isDmaTransferOnGoing = true;
    NVIC_SetPriority(DMA0_DMA16_IRQn, 1);            // 设置DMA中断优先级 范围0-15 越小优先级越高
}

void spi_lcd_fill_area(lcd_display_t *lcd, uint16_t *buf, uint32_t length)
{
    LPSPI_Type *spi = lcd->spi_base;
    LPSPI_Wait_Idle(spi);
    spi->TCR = LPSPI_16Bit_TCR(spi->TCR) | LPSPI_TCR_CONT_MASK;
    for (; length; length--) {
        LPSPI_Write_Data(spi, *buf++);
    }
    while (spi->FSR & LPSPI_FSR_TXCOUNT_MASK)
        __NOP();	// Wait FIFO empty
    LPSPI_DATA8b_TCR(spi->TCR);				// Write a new TCR to start a new frame ==> deassert PCS
}

void spi_lcd_reset(lcd_display_t *lcd)
{
    mp_hal_pin_low(lcd->rst_pin);
    mp_hal_delay_ms(150);           // Wait 150ms
    mp_hal_pin_high(lcd->rst_pin);
    mp_hal_delay_ms(50);            // Wait 50ms
}

void spi_lcd_screen_init(lcd_display_t *lcd)
{
	LPSPI_Type *spi = lcd->spi_base;

	LPSPI_FlushFifo(spi, true, true);
	LPSPI_ClearStatusFlags(spi, kLPSPI_AllStatusFlag);

	spi_lcd_reset(lcd);

    if (!PRE_DEFINED_SCREEN(lcd) || lcd->user_ops.screen_init)
        lcd->user_ops.screen_init(lcd->user_ops.user_obj);     // Call user defined function
    else {
        lcd_drivers[lcd->lcd_type].lcd_init(lcd);
    }
    spi_lcd_set_mode(lcd, DEFAULT_ORIENTATION);
}

void spi_lcd_draw_line (lcd_display_t *lcd, Rect_t *rect, uint16_t color, uint16_t thick)
{
#define xstart	(rect->x0)
#define xend	(rect->width)
#define ystart	(rect->y0)
#define yend	(rect->height)
	int dx = xend - xstart;
	int dy = yend - ystart;
	int xadd = 0, yadd = 0;
	uint16_t xpoint = xstart;
	uint16_t ypoint = ystart;

	if (dx > 0)
		xadd = 1;
	else
		if (dx) {
			xadd = -1;
			dx = -dx;
		}
	if (dy > 0)
		yadd = 1;
	else
		if (dy) {
			yadd = -1;
			dy = -dy;
		}

	//Cumulative error
	int distance = (dx < dy) ? dy : dx;
	int xerr = 0;
	int yerr = 0;

	if (thick == 0)
		thick = 1;
	for (int dots = 0; dots < distance; dots++) {
		Rect_t point = {xpoint, ypoint, thick, thick};
		spi_lcd_set_window(lcd, &point);
		spi_lcd_fill_color(lcd, color, thick * thick);

		xerr += dx;
		yerr += dy;
		if (xerr > distance) {
			xerr -= distance;
			xpoint += xadd;
		}
		if (yerr > distance) {
			yerr -= distance;
			ypoint += yadd;
		}
	}
}
