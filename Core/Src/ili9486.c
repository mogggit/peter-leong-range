#include "ili9486.h"
#include "main.h"

// Internal state variables
static uint16_t _width = ILI9486_TFTWIDTH;
static uint16_t _height = ILI9486_TFTHEIGHT;

// --- Low-Level Hardware Macros ---

#define PIN_LOW(PORT, PIN)    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET)
#define PIN_HIGH(PORT, PIN)   HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET)
#define LCD_RS_HIGH() HAL_GPIO_WritePin(TFT_RS_GPIO_Port, TFT_RS_Pin, GPIO_PIN_SET)
#define LCD_RS_LOW()  HAL_GPIO_WritePin(TFT_RS_GPIO_Port, TFT_RS_Pin, GPIO_PIN_RESET)
#define LCD_CS_HIGH() HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET)
#define LCD_CS_LOW()  HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET)

// Writes 8 bits to the scattered data bus pins using main.h definitions
static inline void WRITE_8(uint8_t val) {
    // For each pin, we either Set (lower 16 bits of BSRR) or Reset (upper 16 bits of BSRR)
    TFT_D0_GPIO_Port->BSRR = (val & 0x01) ? TFT_D0_Pin : (uint32_t)TFT_D0_Pin << 16;
    TFT_D1_GPIO_Port->BSRR = (val & 0x02) ? TFT_D1_Pin : (uint32_t)TFT_D1_Pin << 16;
    TFT_D2_GPIO_Port->BSRR = (val & 0x04) ? TFT_D2_Pin : (uint32_t)TFT_D2_Pin << 16;
    TFT_D3_GPIO_Port->BSRR = (val & 0x08) ? TFT_D3_Pin : (uint32_t)TFT_D3_Pin << 16;
    TFT_D4_GPIO_Port->BSRR = (val & 0x10) ? TFT_D4_Pin : (uint32_t)TFT_D4_Pin << 16;
    TFT_D5_GPIO_Port->BSRR = (val & 0x20) ? TFT_D5_Pin : (uint32_t)TFT_D5_Pin << 16;
    TFT_D6_GPIO_Port->BSRR = (val & 0x40) ? TFT_D6_Pin : (uint32_t)TFT_D6_Pin << 16;
    TFT_D7_GPIO_Port->BSRR = (val & 0x80) ? TFT_D7_Pin : (uint32_t)TFT_D7_Pin << 16;
}

static inline void pulseWR(void) {
    PIN_LOW(TFT_WR_GPIO_Port, TFT_WR_Pin);
    __asm("nop"); // Small delay
    PIN_HIGH(TFT_WR_GPIO_Port, TFT_WR_Pin);
}

static inline void write8(uint8_t val) {
    WRITE_8(val);
    pulseWR();
}

static inline void writeCommand(uint8_t cmd) {
    PIN_LOW(TFT_RS_GPIO_Port, TFT_RS_Pin);
    write8(cmd);
}

static inline void writeData(uint8_t data) {
    PIN_HIGH(TFT_RS_GPIO_Port, TFT_RS_Pin);
    write8(data);
}

static inline void reset(void) {
    PIN_LOW(TFT_RST_GPIO_Port, TFT_RST_Pin);
    HAL_Delay(20);
    PIN_HIGH(TFT_RST_GPIO_Port, TFT_RST_Pin);
    HAL_Delay(120);
}

static inline void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    writeCommand(0x2A);
    writeData(x0 >> 8); writeData(x0 & 0xFF);
    writeData(x1 >> 8); writeData(x1 & 0xFF);

    writeCommand(0x2B);
    writeData(y0 >> 8); writeData(y0 & 0xFF);
    writeData(y1 >> 8); writeData(y1 & 0xFF);

    writeCommand(0x2C);
}

static void writeData16(uint16_t data, uint32_t count) {
    uint8_t hi = data >> 8;
    uint8_t lo = data & 0xFF;
    
    PIN_HIGH(TFT_RS_GPIO_Port, TFT_RS_Pin); // Notice: Replaced CD with RS based on your label

    // Optimized loop unrolling for faster fills
    while (count >= 8) {
        write8(hi); write8(lo);
        write8(hi); write8(lo);
        write8(hi); write8(lo);
        write8(hi); write8(lo);
        write8(hi); write8(lo);
        write8(hi); write8(lo);
        write8(hi); write8(lo);
        write8(hi); write8(lo);
        count -= 8;
    }
    while (count--) {
        write8(hi); 
        write8(lo); 
    }
}

// --- Public API Functions ---

void ILI9486_Init(void) {
    // Note: Assuming GPIOs are initialized in main.c via MX_GPIO_Init()
    
    PIN_HIGH(TFT_RD_GPIO_Port, TFT_RD_Pin);
    PIN_LOW(TFT_CS_GPIO_Port, TFT_CS_Pin);

    reset();

    writeCommand(0x11); // Sleep out
    HAL_Delay(120);
    
    writeCommand(0xD0); // Power Setting
    writeData(0x07);
    writeData(0x42);
    writeData(0x18);
    
    writeCommand(0xD1); // VCOM Control
    writeData(0x00);
    writeData(0x07);
    writeData(0x10);
    
    writeCommand(0x36); // Memory Access Control
    writeData(0x40);    // MX, RGB
    
    writeCommand(0x3A); // Interface Pixel Format
    writeData(0x55);    // 16-bit/pixel
    
    writeCommand(0xC5); // Frame Rate Control
    writeData(0x10);
    
    writeCommand(0xC8); // Gamma Adjustment
    writeData(0x00); writeData(0x32); writeData(0x36);
    writeData(0x45); writeData(0x06); writeData(0x16);
    writeData(0x37); writeData(0x75); writeData(0x77);
    writeData(0x54); writeData(0x0C); writeData(0x00);
    
    writeCommand(0x13); // Normal Display Mode
    writeCommand(0x29); // Display ON
    HAL_Delay(50);
}

void ILI9486_FillScreen(uint16_t color) {
    setAddrWindow(0, 0, _width - 1, _height - 1);
    writeData16(color, (uint32_t)_width * _height);
}

void ILI9486_DrawPixel(int16_t x, int16_t y, uint16_t color) {
    if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;
    setAddrWindow(x, y, x, y);
    writeData(color >> 8);
    writeData(color & 0xFF);
}

void ILI9486_SetRotation(uint8_t r) {
    writeCommand(0x36);
    switch (r) {
        case 0:
            writeData(0x48);
            _width  = ILI9486_TFTWIDTH;
            _height = ILI9486_TFTHEIGHT;
            break;
        case 1:
            writeData(0x28);
            _width  = ILI9486_TFTHEIGHT;
            _height = ILI9486_TFTWIDTH;
            break;
        case 2:
            writeData(0x88);
            _width  = ILI9486_TFTWIDTH;
            _height = ILI9486_TFTHEIGHT;
            break;
        case 3:
            writeData(0xE8);
            _width  = ILI9486_TFTHEIGHT;
            _height = ILI9486_TFTWIDTH;
            break;
    }
}

void ILI9486_InvertDisplay(bool i) {
    writeCommand(i ? 0x21 : 0x20);
}

uint16_t ILI9486_ColorRGB(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void ILI9486_DrawChar(uint16_t x, uint16_t y, char ch, FontDef_t font, uint16_t color, uint16_t bgcolor) {
    uint32_t i, b, j;

    // Set the window for the character to speed up transmission
    setAddrWindow(x, y, x + font.FontWidth - 1, y + font.FontHeight - 1);

    // Loop through every row of the character
    for (i = 0; i < font.FontHeight; i++) {
        // Get the row data for the specific character
        // Standard fonts usually start at ASCII 32 (Space)
        b = font.data[(ch - 32) * font.FontHeight + i];

        // Loop through every column (pixel) in that row
        for (j = 0; j < font.FontWidth; j++) {
            // Check if the specific bit is set (starting from the most significant bit)
            if ((b << j) & 0x8000) {
                writeData(color >> 8);
                writeData(color & 0xFF);
            } else {
                writeData(bgcolor >> 8);
                writeData(bgcolor & 0xFF);
            }
        }
    }
}
void ILI9486_DrawString(uint16_t x, uint16_t y, const char* str, FontDef_t font, uint16_t color, uint16_t bgcolor) {
    while (*str) {
        // Check if there is enough space on the screen for the next character
        if (x + font.FontWidth >= _width) {
            x = 0;            // Reset to left margin
            y += font.FontHeight; // Move to next line
        }

        // Stop drawing if we exceed the screen height
        if (y + font.FontHeight >= _height) {
            break;
        }

        ILI9486_DrawChar(x, y, *str, font, color, bgcolor);
        x += font.FontWidth; // Advance cursor for next character
        str++;
    }
}

/**
 * @brief Draws a radar-like grid with a black background and light green lines.
 * @param step The distance in pixels between the grid lines (e.g., 30 or 40).
 */
void ILI9486_DrawRadarGrid(uint16_t step) {
    // 1. Clear screen to Black
    ILI9486_FillScreen(ILI9486_ColorRGB(0, 0, 0));

    // Define a Light Green color (R: 50, G: 200, B: 50)
    uint16_t gridColor = ILI9486_ColorRGB(50, 200, 50);

    // 2. Draw Vertical Lines
    for (uint16_t x = 0; x < _width; x += step) {
        // Draw a vertical line from top to bottom
        for (uint16_t y = 0; y < _height; y++) {
            ILI9486_DrawPixel(x, y, gridColor);
        }
    }

    // 3. Draw Horizontal Lines
    for (uint16_t y = 0; y < _height; y += step) {
        // Draw a horizontal line from left to right
        for (uint16_t x = 0; x < _width; x++) {
            ILI9486_DrawPixel(x, y, gridColor);
        }
    }
    
    // 4. Optional: Draw a "Center Cross" in a brighter green to look like a scope
    uint16_t centerX = _width / 2;
    uint16_t centerY = _height / 2;
    uint16_t brightGreen = ILI9486_ColorRGB(100, 255, 100);
    
    for(uint16_t i = 0; i < _width; i++) ILI9486_DrawPixel(i, centerY, brightGreen);
    for(uint16_t i = 0; i < _height; i++) ILI9486_DrawPixel(centerX, i, brightGreen);
}

/**
 * @brief Erases a rectangular area and restores the radar grid lines within it.
 * @param x, y The top-left corner of the text box.
 * @param w, h The width and height of the text box.
 * @param step The grid spacing used in your radar.
 */
void ILI9486_RestoreGridArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t step) {
    uint16_t gridColor = ILI9486_ColorRGB(50, 200, 50);
    uint16_t black = ILI9486_ColorRGB(0, 0, 0);

    // Set the drawing window to the specific text area
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    PIN_HIGH(TFT_RS_GPIO_Port, TFT_RS_Pin);

    for (uint16_t currY = y; currY < y + h; currY++) {
        for (uint16_t currX = x; currX < x + w; currX++) {
            // Mathematical check: Is this pixel on a vertical or horizontal line?
            if ((currX % step == 0) || (currY % step == 0)) {
                write8(gridColor >> 8);
                write8(gridColor & 0xFF);
            } else {
                write8(black >> 8);
                write8(black & 0xFF);
            }
        }
    }
}

void ILI9486_EraseStringWithGrid(uint16_t x, uint16_t y, const char* str, FontDef_t font, uint16_t step) {
    if (str == NULL) return;

    // 1. Calculate the width of the string in pixels
    uint16_t strLen = strlen(str);
    uint16_t totalWidth = strLen * font.FontWidth;
    uint16_t totalHeight = font.FontHeight;

    // 2. Call the mathematical restoration function for this specific box
    // This effectively "paints" the background grid back over the letters
    ILI9486_RestoreGridArea(x, y, totalWidth, totalHeight, step);
}

void ILI9486_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *image) {
    if ((x + w > _width) || (y + h > _height)) return;

    setAddrWindow(x, y, x + w - 1, y + h - 1);

    PIN_HIGH(TFT_RS_GPIO_Port, TFT_RS_Pin); // Data mode

    uint32_t total = (uint32_t)w * h;
    for (uint32_t i = 0; i < total; i++) {
        write8(image[i] >> 8);
        write8(image[i] & 0xFF);
    }
}