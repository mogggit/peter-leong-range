#include "ili9486.h"
#include "main.h"

// Internal state variables
static uint16_t _width = ILI9486_TFTWIDTH;
static uint16_t _height = ILI9486_TFTHEIGHT;

// --- Low-Level Hardware Macros ---

#define PIN_LOW(PORT, PIN)    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET)
#define PIN_HIGH(PORT, PIN)   HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET)

// Writes 8 bits to the scattered data bus pins using main.h definitions
static inline void WRITE_8(uint8_t val) {
    HAL_GPIO_WritePin(TFT_D0_GPIO_Port, TFT_D0_Pin, (val & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TFT_D1_GPIO_Port, TFT_D1_Pin, (val & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TFT_D2_GPIO_Port, TFT_D2_Pin, (val & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TFT_D3_GPIO_Port, TFT_D3_Pin, (val & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TFT_D4_GPIO_Port, TFT_D4_Pin, (val & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TFT_D5_GPIO_Port, TFT_D5_Pin, (val & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TFT_D6_GPIO_Port, TFT_D6_Pin, (val & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TFT_D7_GPIO_Port, TFT_D7_Pin, (val & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
    writeData(0x48);    // MX, BGR
    
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