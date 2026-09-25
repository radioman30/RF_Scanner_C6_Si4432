// display.h - ST7789 172x320 pe SPI partajat, framebuffer RGB565 + font 5x7
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"

// Landscape (rotit 90 grade la stanga): 320 latime x 172 inaltime
#define LCD_W 320
#define LCD_H 172

// culori RGB565
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_RED     0xF800
#define C_GREEN   0x07E0
#define C_BLUE    0x001F
#define C_YELLOW  0xFFE0
#define C_CYAN    0x07FF
#define C_ORANGE  0xFD20
#define C_GRAY    0x8410
#define C_DGRAY   0x39E7

bool disp_init(spi_host_device_t host, int dc, int cs, int rst, int bl);
void disp_clear(uint16_t color);
void disp_fill(int x, int y, int w, int h, uint16_t color);
void disp_char(int x, int y, char c, uint16_t fg, uint16_t bg, int scale);
void disp_text(int x, int y, const char *s, uint16_t fg, uint16_t bg, int scale);
void disp_flush(void);   // trimite tot framebuffer-ul la ecran
void disp_set_backlight(uint8_t percent);  // 0..100 % luminozitate (PWM)
