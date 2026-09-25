#include "display.h"
#include <string.h>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static int s_bl_gpio = -1;
static bool s_bl_pwm = false;

static const char *TAG = "disp";
static esp_lcd_panel_handle_t s_panel;
static uint16_t *s_fb;   // framebuffer LCD_W*LCD_H

// ---- Font 5x7, MAJUSCULE + cifre + simboluri. Fiecare glif = 7 octeti (biti 4..0) ----
static const char *FONT_CHARS = " 0123456789.-/:%ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static const uint8_t FONT[][7] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, // 0
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, // 1
    {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}, // 2
    {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E}, // 3
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, // 4
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}, // 5
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, // 6
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, // 7
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, // 8
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}, // 9
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}, // .
    {0x00,0x00,0x00,0x0E,0x00,0x00,0x00}, // -
    {0x01,0x02,0x02,0x04,0x08,0x08,0x10}, // /
    {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00}, // :
    {0x19,0x1A,0x02,0x04,0x08,0x0B,0x13}, // %
    {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}, // A
    {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}, // B
    {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}, // C
    {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}, // D
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}, // E
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}, // F
    {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E}, // G
    {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}, // H
    {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}, // I
    {0x07,0x02,0x02,0x02,0x12,0x12,0x0C}, // J
    {0x11,0x12,0x14,0x18,0x14,0x12,0x11}, // K
    {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}, // L
    {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}, // M
    {0x11,0x19,0x15,0x13,0x11,0x11,0x11}, // N
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}, // O
    {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}, // P
    {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}, // Q
    {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}, // R
    {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}, // S
    {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}, // T
    {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}, // U
    {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}, // V
    {0x11,0x11,0x11,0x15,0x15,0x1B,0x11}, // W
    {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}, // X
    {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}, // Y
    {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}, // Z
};

static int glyph_index(char c)
{
    if (c >= 'a' && c <= 'z') c -= 32;       // mapare la majuscule
    const char *p = strchr(FONT_CHARS, c);
    return p ? (int)(p - FONT_CHARS) : -1;
}

static inline void put_px(int x, int y, uint16_t col)
{
    if (x < 0 || y < 0 || x >= LCD_W || y >= LCD_H) return;
    s_fb[y * LCD_W + x] = col;
}

void disp_fill(int x, int y, int w, int h, uint16_t color)
{
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++)
            put_px(x + i, y + j, color);
}

void disp_clear(uint16_t color)
{
    for (int i = 0; i < LCD_W * LCD_H; i++) s_fb[i] = color;
}

void disp_char(int x, int y, char c, uint16_t fg, uint16_t bg, int scale)
{
    int gi = glyph_index(c);
    for (int row = 0; row < 7; row++) {
        uint8_t bits = (gi >= 0) ? FONT[gi][row] : 0;
        for (int col = 0; col < 5; col++) {
            uint16_t used = (bits & (1 << (4 - col))) ? fg : bg;
            disp_fill(x + col * scale, y + row * scale, scale, scale, used);
        }
    }
}

void disp_text(int x, int y, const char *s, uint16_t fg, uint16_t bg, int scale)
{
    int cx = x;
    while (*s) {
        disp_char(cx, y, *s, fg, bg, scale);
        cx += 6 * scale;   // 5 lățime + 1 spațiu
        s++;
    }
}

void disp_flush(void)
{
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, LCD_W, LCD_H, s_fb);
}

bool disp_init(spi_host_device_t host, int dc, int cs, int rst, int bl)
{
    s_fb = heap_caps_malloc(LCD_W * LCD_H * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!s_fb) { ESP_LOGE(TAG, "framebuffer alloc esuat"); return false; }

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = dc,
        .cs_gpio_num = cs,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)host, &io_cfg, &io) != ESP_OK) return false;

    esp_lcd_panel_dev_config_t pcfg = {
        .reset_gpio_num = rst,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    if (esp_lcd_new_panel_st7789(io, &pcfg, &s_panel) != ESP_OK) return false;

    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, true);   // ST7789: inversia culorilor pornită
    // Rotire 90 grade la stanga (landscape): swap_xy + mirror pe Y.
    // Daca iese invers/rasturnat, schimba flag-urile mirror (false<->true).
    esp_lcd_panel_swap_xy(s_panel, true);
    esp_lcd_panel_mirror(s_panel, false, true);
    esp_lcd_panel_set_gap(s_panel, 0, 34);       // offset 34 pe axa de 172 px (dupa swap)
    esp_lcd_panel_disp_on_off(s_panel, true);

    if (bl >= 0) {
        s_bl_gpio = bl;
        ledc_timer_config_t lt = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 5000,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        if (ledc_timer_config(&lt) == ESP_OK) {
            ledc_channel_config_t lc = {
                .gpio_num = bl, .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0,
                .duty = 0, .hpoint = 0,
            };
            if (ledc_channel_config(&lc) == ESP_OK) s_bl_pwm = true;
        }
        if (!s_bl_pwm) {   // rezerva: pornit fix daca PWM esueaza
            gpio_config_t b = { .pin_bit_mask = (1ULL << bl), .mode = GPIO_MODE_OUTPUT };
            gpio_config(&b);
            gpio_set_level(bl, 1);
        }
    }
    disp_clear(C_BLACK);
    disp_flush();
    return true;
}

void disp_set_backlight(uint8_t percent)
{
    if (percent > 100) percent = 100;
    if (s_bl_pwm) {
        uint32_t duty = (uint32_t)percent * 255 / 100;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else if (s_bl_gpio >= 0) {
        gpio_set_level(s_bl_gpio, percent >= 50 ? 1 : 0);
    }
}
