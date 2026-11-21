// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: Copyright (C) 2026 iamankushpandit <https://github.com/iamankushpandit>
//
// Part of Ivy AI -- https://github.com/iamankushpandit/esp_ai
// Free software under GPL-3.0-or-later, with the Espressif SDK linking
// exception in LICENSE.exception. Reusing any part of this file, in any
// work, must keep this notice, credit iamankushpandit as the author,
// and stay under the same licence with corresponding source offered.
// See LICENSE, LICENSE.exception, NOTICE.md and THIRD_PARTY.md.

// NV3023 296x240 landscape over SPI3 via esp_lcd panel IO (TP-01 / ostb-3st).
// No framebuffer: callers stream pixels through two small ping-pong DMA buffers.
// Init sequence matches fw-kira/ostb-3st.cc exactly (BGR, invert OFF, offset_x=24).
#include "board.h"
#include <math.h>
#include "board_lcd_stream.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_io.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "lcd";

#define LCD_HOST SPI3_HOST
#define LCD_HZ (40 * 1000 * 1000)

static esp_lcd_panel_io_handle_t s_io;
static uint16_t *s_buf[2];
static int s_next;
static SemaphoreHandle_t s_free;   // counts free DMA buffers
static bool s_first_chunk;

static bool on_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *e, void *ctx)
{
    BaseType_t woke = pdFALSE;
    xSemaphoreGiveFromISR(s_free, &woke);
    return woke == pdTRUE;
}

static void cmd(uint8_t c, const uint8_t *p, size_t n) { esp_lcd_panel_io_tx_param(s_io, c, p, n); }
#define CMD(c, ...) do { static const uint8_t d_[] = {__VA_ARGS__}; cmd(c, d_, sizeof d_); } while (0)

static void wait_idle(void)
{
    for (int i = 0; i < 2; i++) xSemaphoreTake(s_free, portMAX_DELAY);
    for (int i = 0; i < 2; i++) xSemaphoreGive(s_free);
}

esp_err_t board_lcd_init(void)
{
    gpio_config_t rst = {
        .pin_bit_mask = 1ULL << BOARD_LCD_RST,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rst);
    gpio_set_level(BOARD_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(BOARD_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    spi_bus_config_t bus = {
        .mosi_io_num = BOARD_LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = BOARD_LCD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BOARD_LCD_STREAM_PX * 2,
    };
    ESP_RETURN_ON_ERROR_LOG(spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io = {
        .dc_gpio_num = BOARD_LCD_DC,
        .cs_gpio_num = BOARD_LCD_CS,
        .pclk_hz = LCD_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 4,
        .on_color_trans_done = on_done,
    };
    ESP_RETURN_ON_ERROR_LOG(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io, &s_io));

    for (int i = 0; i < 2; i++) {
        s_buf[i] = heap_caps_malloc(BOARD_LCD_STREAM_PX * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_buf[i]) {
            ESP_LOGE(TAG, "DMA line buffer alloc failed (%d B)", BOARD_LCD_STREAM_PX * 2);
            return ESP_ERR_NO_MEM;
        }
    }
    s_free = xSemaphoreCreateCounting(2, 2);

    // NV3023 init sequence from fw-kira (ostb-3st.cc)
    CMD(0xFF, 0xA5);
    CMD(0x3E, 0x09);
    CMD(0x3A, 0x65);
    CMD(0x82, 0x00);
    CMD(0x98, 0x00);
    CMD(0x63, 0x0F);
    CMD(0x64, 0x0F);
    CMD(0xB4, 0x34);
    CMD(0xB5, 0x30);
    CMD(0x83, 0x03);
    CMD(0x86, 0x04);
    CMD(0x87, 0x16);
    CMD(0x88, 0x0A);
    CMD(0x89, 0x27);
    CMD(0x93, 0x63);
    CMD(0x96, 0x81);
    CMD(0xC3, 0x10);
    CMD(0xE6, 0x00);
    CMD(0x99, 0x01);
    CMD(0x70, 0x09);
    CMD(0x71, 0x1D);
    CMD(0x72, 0x14);
    CMD(0x73, 0x0A);
    CMD(0x74, 0x11);
    CMD(0x75, 0x16);
    CMD(0x76, 0x38);
    CMD(0x77, 0x0B);
    CMD(0x78, 0x08);
    CMD(0x79, 0x3E);
    CMD(0x7A, 0x07);
    CMD(0x7B, 0x0D);
    CMD(0x7C, 0x16);
    CMD(0x7D, 0x0F);
    CMD(0x7E, 0x14);
    CMD(0x7F, 0x05);
    CMD(0xA0, 0x04);
    CMD(0xA1, 0x28);
    CMD(0xA2, 0x0C);
    CMD(0xA3, 0x11);
    CMD(0xA4, 0x0B);
    CMD(0xA5, 0x23);
    CMD(0xA6, 0x45);
    CMD(0xA7, 0x07);
    CMD(0xA8, 0x0A);
    CMD(0xA9, 0x3B);
    CMD(0xAA, 0x0D);
    CMD(0xAB, 0x18);
    CMD(0xAC, 0x14);
    CMD(0xAD, 0x0F);
    CMD(0xAE, 0x19);
    CMD(0xAF, 0x08);
    CMD(0xFF, 0x00);
    // MADCTL: swap_xy=1 (0x20) | mirror_x=1 (0x40) | mirror_y=1 (0x80) | BGR (0x08) = 0xE8
    CMD(0x36, 0xE8);
    CMD(0x3A, 0x55);  // 16-bit RGB565
    cmd(0x11, NULL, 0);  // sleep out
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(0x20, NULL, 0);  // inversion OFF (ostb-3st.cc: invert_color false)
    board_lcd_fill(0, 0, BOARD_LCD_W, BOARD_LCD_H, 0x0000);
    cmd(0x29, NULL, 0);  // display on (after clearing, so no garbage flash)
    vTaskDelay(pdMS_TO_TICKS(10));

    // Backlight: LEDC 5 kHz, 8-bit, active high. Starts off.
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR_LOG(ledc_timer_config(&t));
    ledc_channel_config_t ch = {
        .gpio_num = BOARD_LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ESP_RETURN_ON_ERROR_LOG(ledc_channel_config(&ch));
    ESP_LOGI(TAG, "NV3023 296x240 ready @%d MHz", LCD_HZ / 1000000);
    return ESP_OK;
}

void board_backlight(uint8_t pct)
{
    if (pct > 100) pct = 100;
    uint32_t duty = pct ? (uint32_t)lroundf(powf(pct / 100.0f, 2.2f) * 255.0f) : 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty ? duty : (pct ? 1 : 0));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void board_lcd_window(int x, int y, int w, int h)
{
    wait_idle();
    int x0 = x + BOARD_LCD_OFFSET_X, x1 = x + w - 1 + BOARD_LCD_OFFSET_X;
    int y0 = y + BOARD_LCD_OFFSET_Y, y1 = y + h - 1 + BOARD_LCD_OFFSET_Y;
    uint8_t ca[4] = {(uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF), (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF)};
    uint8_t ra[4] = {(uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF), (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF)};
    cmd(0x2A, ca, 4);
    cmd(0x2B, ra, 4);
    s_first_chunk = true;
}

uint16_t *board_lcd_stream_buf(void)
{
    xSemaphoreTake(s_free, portMAX_DELAY);
    uint16_t *b = s_buf[s_next];
    s_next ^= 1;
    return b;
}

void board_lcd_stream_push(uint16_t *buf, size_t npx)
{
    esp_lcd_panel_io_tx_color(s_io, s_first_chunk ? 0x2C : 0x3C, buf, npx * 2);
    s_first_chunk = false;
}

void board_lcd_stream_end(void) { wait_idle(); }

static inline uint16_t swap16(uint16_t c) { return (uint16_t)((c >> 8) | (c << 8)); }

void board_lcd_fill(int x, int y, int w, int h, uint16_t color)
{
    if (w <= 0 || h <= 0) return;
    board_lcd_window(x, y, w, h);
    size_t total = (size_t)w * h;
    uint16_t sc = swap16(color);
    while (total) {
        size_t n = total < BOARD_LCD_STREAM_PX ? total : BOARD_LCD_STREAM_PX;
        uint16_t *b = board_lcd_stream_buf();
        for (size_t i = 0; i < n; i++) b[i] = sc;
        board_lcd_stream_push(b, n);
        total -= n;
    }
    board_lcd_stream_end();
}

void board_lcd_blit(int x, int y, int w, int h, const uint16_t *px)
{
    if (w <= 0 || h <= 0) return;
    board_lcd_window(x, y, w, h);
    size_t total = (size_t)w * h;
    while (total) {
        size_t n = total < BOARD_LCD_STREAM_PX ? total : BOARD_LCD_STREAM_PX;
        uint16_t *b = board_lcd_stream_buf();
        for (size_t i = 0; i < n; i++) b[i] = swap16(px[i]);
        board_lcd_stream_push(b, n);
        px += n;
        total -= n;
    }
    board_lcd_stream_end();
}
