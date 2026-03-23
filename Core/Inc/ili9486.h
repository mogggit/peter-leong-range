#ifndef ILI9486_H
#define ILI9486_H

#include <stdint.h>
#include <stdbool.h>

/* IMPORTANT: Change this include to match your specific STM32 family 
   (e.g., stm32f4xx_hal.h, stm32g0xx_hal.h, etc.) */
#include "stm32f4xx_hal.h"

// Screen dimensions
#define ILI9486_TFTWIDTH  320
#define ILI9486_TFTHEIGHT 480

// Function Prototypes
void ILI9486_Init(void);
void ILI9486_FillScreen(uint16_t color);
void ILI9486_DrawPixel(int16_t x, int16_t y, uint16_t color);
void ILI9486_SetRotation(uint8_t r);
void ILI9486_InvertDisplay(bool i);

// Helper macro for 565 color format
uint16_t ILI9486_ColorRGB(uint8_t r, uint8_t g, uint8_t b);

#endif // ILI9486_H