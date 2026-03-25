#ifndef ILI9486_H
#define ILI9486_H

#include <stdint.h>
#include <stdbool.h>

/* IMPORTANT: Change this include to match your specific STM32 family 
   (e.g., stm32f4xx_hal.h, stm32g0xx_hal.h, etc.) */
#include "stm32f4xx_hal.h"
#include "fonts.h"

// Screen dimensions
#define ILI9486_TFTWIDTH  320
#define ILI9486_TFTHEIGHT 480

// Colors
#define BLACK       0x0000
#define WHITE       0xFFFF
#define RED         0xF800
#define GREEN       0x07E0
#define BLUE        0x001F
#define YELLOW      0xFFE0
#define CYAN        0x07FF
#define MAGENTA     0xF81F

// Function Prototypes
void ILI9486_Init(void);
void ILI9486_FillScreen(uint16_t color);
void ILI9486_DrawPixel(int16_t x, int16_t y, uint16_t color);
void ILI9486_SetRotation(uint8_t r);
void ILI9486_InvertDisplay(bool i);
void ILI9486_DrawImageFast(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *data);
void ILI9486_DrawChar(uint16_t x, uint16_t y, char ch, FontDef_t font, uint16_t color, uint16_t bgcolor);
void ILI9486_DrawString(uint16_t x, uint16_t y, const char* str, FontDef_t font, uint16_t color, uint16_t bgcolor);
void ILI9486_DrawRadarGrid(uint16_t step);
void ILI9486_RestoreGridArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t step);
void ILI9486_EraseStringWithGrid(uint16_t x, uint16_t y, const char* str, FontDef_t font, uint16_t step);

// Helper macro for 565 color format
uint16_t ILI9486_ColorRGB(uint8_t r, uint8_t g, uint8_t b);

#endif // ILI9486_H