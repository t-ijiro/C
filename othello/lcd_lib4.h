/*
 * lcd_lib4.h
 *
 *  Created on: 2017/07/20
 *      Author: kwatanabe
 */

#ifndef LCD_LIB4_H_
#define LCD_LIB4_H_

#include "iodefine.h"

#define LCD_E	PORTD.PODR.BIT.B3
#define LCD_RS	PORTD.PODR.BIT.B0
#define LCD_RW	PORTD.PODR.BIT.B1
#define LCD_DB	PORTD.PODR.BYTE

//	LCD Mode
#define CMDmode	(LCD_RS = 0)
#define DATmode (LCD_RS = 1)
// LCD Instruction
#define LCD_INIT8B	0x03
#define LCD_INIT4B	0x02
#define LCD_FCSET4B	0x28
#define	LCD_DISP_OFF	0x08
#define LCD_CLEAR	0x01
#define LCD_ENTSET	0x06
#define	LCD_DISP_NCUR	0x0C

//Prototype declaration of function
void wait50us(unsigned int wait_time);
void flush_lcd(void);
void init_LCD(void);
void lcd_clear(void);
void lcd_put(char c);
void lcd_puts(char *str);
void lcd_xy(unsigned char x, unsigned char y);
void lcd_dataout(unsigned long data);
void set_pattern(void);

//Groval variables
#define BUFFER_SIZE 32
static unsigned char buf[BUFFER_SIZE];
static unsigned char wp, rp;

#define loop_const	225
//#define loop_const	4

void wait50us(unsigned int wait_time){
	volatile unsigned int loop1,loop2;

	for(loop1 = 0; loop1 < wait_time; loop1++){
		for(loop2 = 0; loop2 < loop_const; loop2++);
	}
}

void flush_lcd(void){
	unsigned char data;

	while(wp != rp){
		data = buf[rp];
		rp++;
		if(rp >= BUFFER_SIZE) rp = 0;

		wait50us(1);
		LCD_E = 1;
		LCD_DB = (LCD_DB & 0x0F) | (data & 0xF0);
		LCD_E = 0;
		LCD_E = 0;

		LCD_E = 1;
		LCD_DB = (LCD_DB & 0x0F) | (data << 4);
		LCD_E = 0;
		LCD_E = 0;
	}
}

void init_LCD(void){
	wait50us(1000);
	PORTD.PDR.BYTE = 0xff;
	PORTD.PODR.BYTE &= 0x00;

	wp = rp = 0;
	CMDmode;
	LCD_E = 0;

	LCD_E = 1;
	LCD_DB = (LCD_DB & 0x0F) | (LCD_INIT8B << 4);
	LCD_E = 0;

	wait50us(200);
	LCD_E = 1;
	LCD_DB = (LCD_DB & 0x0F) | (LCD_INIT8B << 4);
	LCD_E = 0;

	wait50us(40);
	LCD_E = 1;
	LCD_DB = (LCD_DB & 0x0F) | (LCD_INIT8B << 4);
	LCD_E = 0;

	wait50us(40);
	LCD_E = 1;
	LCD_DB = (LCD_DB & 0x0F) | (LCD_INIT4B << 4);
	LCD_E = 0;

	wait50us(40);
	lcd_put(LCD_FCSET4B);
	flush_lcd();

	wait50us(40);
	lcd_put(LCD_DISP_OFF);
	flush_lcd();

	wait50us(40);
	lcd_put(LCD_CLEAR);
	flush_lcd();

	wait50us(40);
	lcd_put(LCD_ENTSET);
	flush_lcd();

	wait50us(40);
	lcd_put(LCD_DISP_NCUR);
	flush_lcd();

	set_pattern();

	lcd_clear();
}

//CG-RAMへのキャラクタパターンの書き込み
#define	cgram_start_address 0x40
const char ptn[9][8] = {
		{0x00, 0x00, 0x0F, 0x11, 0x11, 0x0F, 0x01, 0x0E},
		{0x02, 0x00, 0x06, 0x02, 0x02, 0x02, 0x12, 0x0C},
		{0x00, 0x00, 0x1a, 0x15, 0x15, 0x15, 0x11, 0x00},
		{0x00, 0x00, 0x16, 0x19, 0x11, 0x1e, 0x10, 0x10},
		{0x00, 0x00, 0x0D, 0x13, 0x11, 0x0F, 0x01, 0x01},
		{0x00, 0x00, 0x11, 0x11, 0x11, 0x0f, 0x01, 0x0e},

		{0x0E, 0x0A, 0x0E, 0x05, 0x0E, 0x14, 0x0A, 0x11},
		{0x0E, 0x0A, 0x0E, 0x04, 0x04, 0x04, 0x04, 0x04},
		{0x0E, 0x0A, 0x0E, 0x14, 0x0E, 0x05, 0x0A, 0x11}
};
void set_pattern(void){
	unsigned char nc,pr;

	CMDmode;
	lcd_put(cgram_start_address);
	flush_lcd();

	DATmode;
	for(nc = 0; nc < 9; nc++){
		for(pr = 0; pr < 8; pr++){
			lcd_put(ptn[nc][pr]);
		}
		flush_lcd();
	}
}

void lcd_clear(void){
	flush_lcd();
	CMDmode;
	lcd_put(LCD_CLEAR);
	flush_lcd();
	wait50us(40);
	DATmode;
}

void lcd_put(char c){
	switch(c){
		case 'g': c = 0x00; break;
		case 'j': c = 0x01; break;
		case 'm': c = 0x02; break;
		case 'p': c = 0x03; break;
		case 'q': c = 0x04; break;
		case 'y': c = 0x05; break;
	}
	buf[wp] = c;
	wp++;
	if(wp >= BUFFER_SIZE) wp = 0;
}

void lcd_puts(char *str){
	char c;
	while(*str){
		c = *str;
		lcd_put(c);
		str++;
	}
}

void lcd_xy(unsigned char x, unsigned char y){
	char adr;
	flush_lcd();
	CMDmode;
	x--;y--;
	if(x > 0x0f) x = 0x00;
	if(y == 0x01) y = 0x40;
	adr = (x + y) | 0x80;
	lcd_put(adr);
	flush_lcd();
	DATmode;
}

void lcd_dataout(unsigned long data){

	volatile int i = 0;
	char str[16];

	if(!data)
	{
		lcd_put('0');
		flush_lcd();
		return;
	}

	while(data > 0)
	{
		str[i++] = (data % 10) + '0';
		data /= 10;
	}

	for(i = i - 1; i >= 0; --i)
	{
		lcd_put(str[i]);
	}

	flush_lcd();

}

#endif /* LCD_LIB4_H_ */
