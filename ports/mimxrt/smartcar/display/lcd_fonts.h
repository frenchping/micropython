/*
 * drv_lcd_fonts.h
 *
 *  Created on: Feb 18, 2021
 *      Author: nxa19017
 */

#ifndef DRV_LCD_FONTS_H_
#define DRV_LCD_FONTS_H_

typedef enum {
	FONT_CODING_XY_LH = 0,		// Horizontal ==> Vertical. Each byte is LSB->MSB
	FONT_CODING_XY_HL,			// Horizontal ==> Vertical. Each byte is MSB->LSB
} FONT_CODING;

typedef struct _asc_font_t
{
	const unsigned char *Fonts;
	unsigned char Width;
	unsigned char Height;
	unsigned char Bytes;
	unsigned char BytesOnWidth:4;
	unsigned char Coding:4;
} Asc_Font_t;

extern const Asc_Font_t Asc_Font12;
extern const Asc_Font_t Asc_Font16;
extern const Asc_Font_t Asc_Font24;
extern const Asc_Font_t Asc_Font32;
extern const Asc_Font_t Asc_Font64;
extern const Asc_Font_t Clock_Font32;   // Used for display clock

#endif /* DRV_LCD_FONTS_H_ */
