#include "main.h"
#include "drv_lcd.h"
#include "drv_lcd_font.h"
#include <string.h>

/*
 * SECTION 1: FSMC MEMORY MAPPING
 * The STM32 treats the LCD not as a peripheral, but as external MEMORY (SRAM).
 * We write to specific memory addresses to send Commands or Data.
 *
 * Base Address (0x68000000):
 * - Corresponds to FSMC Bank 1, Sector 3 (NE3 pin PG10).
 *
 * Offset (0x40000):
 * - We use Address Line 18 (A18 / PD13) as the "Register Select" (RS) pin.
 * - 2^18 = 262,144 = 0x40000.
 * - Writing to (Base)         -> A18 is Low  -> LCD interprets as COMMAND.
 * - Writing to (Base + 0x40K) -> A18 is High -> LCD interprets as DATA.
 */
#define LCD_BASE        ((uint32_t)0x68000000)
#define LCD_REG         (*((volatile uint8_t *)LCD_BASE))           // Write here for Commands
#define LCD_RAM         (*((volatile uint8_t *)(LCD_BASE + 0x40000))) // Write here for Data

// Global structure to hold LCD parameters
_lcd_dev lcddev;

// Default Colors
uint16_t BACK_COLOR = WHITE;
uint16_t FORE_COLOR = BLACK;

/*
 * SECTION 2: LOW-LEVEL BUS WRITES
 * These functions handle the physical transmission of bytes over the 8-bit
 * parallel bus (D0-D7).
 */

// Write a Command Byte (RS Pin Low)
void LCD_WR_REG(uint8_t reg) {
    LCD_REG = reg;
}

// Write a Data Byte (RS Pin High)
void LCD_WR_DATA8(uint8_t data) {
    LCD_RAM = data;
}

/**
 * @brief  Writes a 16-bit color value to the 8-bit bus.
 * @why    Our hardware bus is only 8 bits wide (D0-D7), but pixels are 16 bits (RGB565).
 * We must manually split the 16-bit integer into two 8-bit transfers.
 * The ST7789 expects the High Byte (MSB) first, then the Low Byte (LSB).
 */
void LCD_WR_DATA16(uint16_t data) {
    LCD_RAM = (data >> 8);   // Shift right to isolate High Byte (e.g., 0xF8 from 0xF800)
    LCD_RAM = (data & 0xFF); // Mask to isolate Low Byte (e.g., 0x00 from 0xF800)
}

// Prepares the LCD to receive pixel data (sends "Write RAM" command 0x2C)
void LCD_WriteRAM_Prepare(void) {
    LCD_WR_REG(lcddev.wramcmd);
}

// Wrapper to write a pixel color
void LCD_WriteRAM(uint16_t color) {
    LCD_WR_DATA16(color);
}

/*
 * SECTION 3: INITIALIZATION SEQUENCE
 * This sequence configures the ST7789 driver chip. These are manufacturer
 * specific "Magic Numbers" that set voltages, gamma curves, and update rates.
 */

void LCD_Reset(void) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(50);
}

void LCD_Backlight_On(void) {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
}

void LCD_Init(void) {
    LCD_Reset();
    LCD_Backlight_On();

    // 1. Wake up
    LCD_WR_REG(0x11); // Sleep Out
    HAL_Delay(120);

    // 2. Orientation & Format (how to interpret data)
    LCD_WR_REG(0x36); LCD_WR_DATA8(0x00); // Memory Access Control (Orientation)
    LCD_WR_REG(0x3A); LCD_WR_DATA8(0x05); // Pixel Format: 0x05 = 16-bit RGB565

    // 3. Power & Gamma Settings (Vendor Specific) - make sure the color is right , white is white, black is black
    LCD_WR_REG(0xB2); LCD_WR_DATA8(0x0C); LCD_WR_DATA8(0x0C); LCD_WR_DATA8(0x00); LCD_WR_DATA8(0x33); LCD_WR_DATA8(0x33);
    LCD_WR_REG(0xB7); LCD_WR_DATA8(0x35);
    LCD_WR_REG(0xBB); LCD_WR_DATA8(0x19);
    LCD_WR_REG(0xC0); LCD_WR_DATA8(0x2C);
    LCD_WR_REG(0xC2); LCD_WR_DATA8(0x01);
    LCD_WR_REG(0xC3); LCD_WR_DATA8(0x12);
    LCD_WR_REG(0xC4); LCD_WR_DATA8(0x20);
    LCD_WR_REG(0xC6); LCD_WR_DATA8(0x0F);
    LCD_WR_REG(0xD0); LCD_WR_DATA8(0xA4); LCD_WR_DATA8(0xA1);

    // Gamma Correction (Makes colors look correct)
    LCD_WR_REG(0xE0); LCD_WR_DATA8(0xD0); LCD_WR_DATA8(0x04); LCD_WR_DATA8(0x0D); LCD_WR_DATA8(0x11); LCD_WR_DATA8(0x13); LCD_WR_DATA8(0x2B); LCD_WR_DATA8(0x3F); LCD_WR_DATA8(0x54); LCD_WR_DATA8(0x4C); LCD_WR_DATA8(0x18); LCD_WR_DATA8(0x0D); LCD_WR_DATA8(0x0B); LCD_WR_DATA8(0x1F); LCD_WR_DATA8(0x23);
    LCD_WR_REG(0xE1); LCD_WR_DATA8(0xD0); LCD_WR_DATA8(0x04); LCD_WR_DATA8(0x0C); LCD_WR_DATA8(0x11); LCD_WR_DATA8(0x13); LCD_WR_DATA8(0x2C); LCD_WR_DATA8(0x3F); LCD_WR_DATA8(0x44); LCD_WR_DATA8(0x51); LCD_WR_DATA8(0x2F); LCD_WR_DATA8(0x1F); LCD_WR_DATA8(0x1F); LCD_WR_DATA8(0x20); LCD_WR_DATA8(0x23);

    // 4. Turn On
    LCD_WR_REG(0x21); // Display Inversion On (Fixes inverted colors on ST7789)
    LCD_WR_REG(0x29); // Display ON

    // 5. Set Software Parameters
    lcddev.width = 240;
    lcddev.height = 240;
    lcddev.setxcmd = 0x2A; // Column Address Set
    lcddev.setycmd = 0x2B; // Row Address Set
    lcddev.wramcmd = 0x2C; // Write Memory Start

    LCD_Clear(WHITE); // Clear screen garbage data
}

/*
 * SECTION 4: DRAWING LOGIC
 */

// Defines the active rectangular area we are writing to
void LCD_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    LCD_WR_REG(lcddev.setxcmd); // Set X coordinates
    LCD_WR_DATA8(x1 >> 8); LCD_WR_DATA8(x1 & 0xFF); // Start X
    LCD_WR_DATA8(x2 >> 8); LCD_WR_DATA8(x2 & 0xFF); // End X

    LCD_WR_REG(lcddev.setycmd); // Set Y coordinates
    LCD_WR_DATA8(y1 >> 8); LCD_WR_DATA8(y1 & 0xFF); // Start Y
    LCD_WR_DATA8(y2 >> 8); LCD_WR_DATA8(y2 & 0xFF); // End Y

    LCD_WriteRAM_Prepare(); // Ready to write pixel data
}

// Fills the entire screen with a single color
void LCD_Clear(uint16_t color) {
    LCD_SetWindow(0, 0, lcddev.width-1, lcddev.height-1); // Select full screen
    uint32_t total_pixels = lcddev.width * lcddev.height;

    // Burst write the color to every pixel
    for (uint32_t i = 0; i < total_pixels; i++) {
        LCD_WR_DATA16(color);
    }
}

// Draw a single pixel at (x, y)
void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= lcddev.width || y >= lcddev.height) return; // Bounds check
    LCD_SetWindow(x, y, x, y); // Select 1x1 window
    LCD_WR_DATA16(color);      // Write color
}

/*
 * SECTION 5: TEXT RENDERING
 * Uses the font array defined in drv_lcd_font.h to draw text.
 */

// Draws a single character
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t color, uint16_t back_color) {
    uint8_t temp, t;
    uint16_t pos;

    if (x > lcddev.width - 8 || y > lcddev.height - 16) return;

    num = num - ' '; // Adjust ASCII to array index (font starts at space)
    LCD_SetWindow(x, y, x + 7, y + 15); // Set 8x16 window

    // Loop through the 16 rows of the character font
    for (pos = 0; pos < 16; pos++) {
        temp = asc2_1608[num * 16 + pos]; // Get the bit pattern for this row

        // Loop through the 8 pixels in the row
        for (t = 0; t < 8; t++) {
            // Check if bit is 1 (Foreground) or 0 (Background)
            if (temp & 0x80) LCD_WR_DATA16(color);
            else             LCD_WR_DATA16(back_color);
            temp <<= 1; // Shift to next bit
        }
    }
}

// Draws a String
void LCD_ShowString(uint16_t x, uint16_t y, char *p, uint16_t color, uint16_t back_color) {
    while (*p != '\0') { // Loop until null terminator
        // Auto-wrap logic
        if (x > lcddev.width - 8) {
            x = 0;
            y += 16;
        }
        if (y > lcddev.height - 16) break;

        LCD_ShowChar(x, y, *p, color, back_color);
        x += 8; // Move cursor right
        p++;    // Move to next char
    }
}
