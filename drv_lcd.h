#ifndef __DRV_LCD_H
#define __DRV_LCD_H

#include "main.h"

/* LCD Colors */
#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define GREEN       0x07E0
#define YELLOW      0xFFE0
#define GRAY        0X8430

/* LCD Parameter Structure */
typedef struct {
    uint16_t width;
    uint16_t height;
    uint16_t id;
    uint8_t  dir;
    uint16_t wramcmd;
    uint16_t setxcmd;
    uint16_t setycmd;
} _lcd_dev;

extern _lcd_dev lcddev;
extern uint16_t BACK_COLOR;
extern uint16_t FORE_COLOR;

/* Function Prototypes */
void LCD_Init(void);
void LCD_Clear(uint16_t color);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void LCD_ShowString(uint16_t x, uint16_t y, char *p, uint16_t color, uint16_t back_color);
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t color, uint16_t back_color);

#endif
