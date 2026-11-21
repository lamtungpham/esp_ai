// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: Copyright (C) 2026 iamankushpandit <https://github.com/iamankushpandit>
//
// Part of Ivy AI -- https://github.com/iamankushpandit/esp_ai
// Free software under GPL-3.0-or-later, with the Espressif SDK linking
// exception in LICENSE.exception. Reusing any part of this file, in any
// work, must keep this notice, credit iamankushpandit as the author,
// and stay under the same licence with corresponding source offered.
// See LICENSE, LICENSE.exception, NOTICE.md and THIRD_PARTY.md.

// TP-01 (ostb-3st) dual audio codec:
//   - ES8311 (0x18) DAC output (speaker) via I2S0 TX (STD mono -> L+R)
//   - ES7210 (0x40) ADC input (mic) via I2S0 RX (TDM 4-slot, slot 0 = MIC1)
//   - PA_PIN (GPIO 4) active HIGH
//   - 16 kHz, 16-bit, MCLK = 256 * fs = 4.096 MHz
#include "board.h"
#include "board_priv.h"
#include "board_lcd_stream.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "audio";

#define ES8311_ADDR 0x18
#define ES7210_ADDR 0x40
#define DMA_FRAMES 256
#define DMA_DESC 6
#define VOLUME_CAP_PCT 90

static i2c_master_dev_handle_t s_dac;
static i2c_master_dev_handle_t s_adc;
static i2s_chan_handle_t s_tx, s_rx;
static bool s_amp;

static esp_err_t wr_dev(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    esp_err_t e = i2c_master_transmit(dev, b, 2, 100);
    if (e != ESP_OK) ESP_LOGE(TAG, "i2c write reg 0x%02X failed: %s", reg, esp_err_to_name(e));
    return e;
}

static uint8_t rd_dev(i2c_master_dev_handle_t dev, uint8_t reg)
{
    uint8_t v = 0;
    i2c_master_transmit_receive(dev, &reg, 1, &v, 1, 100);
    return v;
}

// ES8311 DAC @ 16 kHz, MCLK = 4.096 MHz (256fs)
static esp_err_t es8311_init(void)
{
    ESP_RETURN_ON_ERROR_LOG(board_i2c_add(ES8311_ADDR, &s_dac));
    ESP_RETURN_ON_ERROR_LOG(wr_dev(s_dac, 0x00, 0x1F));  // reset
    vTaskDelay(pdMS_TO_TICKS(20));
    wr_dev(s_dac, 0x00, 0x00);
    wr_dev(s_dac, 0x00, 0x80);                           // power on, slave mode
    wr_dev(s_dac, 0x01, 0x3F);                           // clocks on, MCLK from pin
    wr_dev(s_dac, 0x02, rd_dev(s_dac, 0x02) & 0x07);     // pre_div = 1, pre_multi = x1
    wr_dev(s_dac, 0x03, 0x10);                           // fs single, adc_osr
    wr_dev(s_dac, 0x04, 0x10);                           // dac_osr
    wr_dev(s_dac, 0x05, 0x00);                           // adc/dac div = 1
    wr_dev(s_dac, 0x06, (rd_dev(s_dac, 0x06) & 0xE0) | 0x03); // bclk_div = 4
    wr_dev(s_dac, 0x07, (rd_dev(s_dac, 0x07) & 0xC0) | 0x00); // lrck_h
    wr_dev(s_dac, 0x08, 0xFF);                           // lrck_l (256)
    wr_dev(s_dac, 0x00, rd_dev(s_dac, 0x00) & 0xBF);     // slave mode
    wr_dev(s_dac, 0x09, 0x0C);                           // SDP in: I2S 16-bit
    wr_dev(s_dac, 0x0A, 0x0C);                           // SDP out: I2S 16-bit
    wr_dev(s_dac, 0x0D, 0x01);                           // analog power up
    wr_dev(s_dac, 0x12, 0x00);                           // DAC power up
    wr_dev(s_dac, 0x13, 0x10);                           // output drive enable
    wr_dev(s_dac, 0x37, 0x08);                           // DAC EQ bypass
    uint8_t id1 = rd_dev(s_dac, 0xFD), id2 = rd_dev(s_dac, 0xFE);
    ESP_LOGI(TAG, "ES8311 chip id %02X%02X", id1, id2);
    return ESP_OK;
}

// ES7210 4-ch TDM ADC @ 16 kHz, MCLK = 4.096 MHz (256fs)
static esp_err_t es7210_init(void)
{
    ESP_RETURN_ON_ERROR_LOG(board_i2c_add(ES7210_ADDR, &s_adc));
    ESP_RETURN_ON_ERROR_LOG(wr_dev(s_adc, 0x00, 0xFF));  // soft reset
    vTaskDelay(pdMS_TO_TICKS(10));
    wr_dev(s_adc, 0x00, 0x32);                           // release reset, power up
    wr_dev(s_adc, 0x09, 0x30);
    wr_dev(s_adc, 0x0A, 0x30);
    wr_dev(s_adc, 0x23, 0x2A);
    wr_dev(s_adc, 0x22, 0x0A);
    wr_dev(s_adc, 0x21, 0x2A);
    wr_dev(s_adc, 0x20, 0x0A);
    wr_dev(s_adc, 0x08, 0x10);                           // slave mode, MCLK from pad
    wr_dev(s_adc, 0x11, 0x60);                           // 16-bit data
    wr_dev(s_adc, 0x12, 0x02);                           // TDM mode enabled
    wr_dev(s_adc, 0x40, 0xC3);                           // clock on
    wr_dev(s_adc, 0x41, 0x70);
    wr_dev(s_adc, 0x42, 0x70);
    // Clock dividers for 16 kHz @ MCLK = 4.096 MHz (256fs)
    wr_dev(s_adc, 0x02, 0xC1);                           // adc_div=1, dll_div=1
    wr_dev(s_adc, 0x03, 0x04);                           // bclk_div=4 (1.024 MHz)
    wr_dev(s_adc, 0x04, 0x01);                           // lrck_h
    wr_dev(s_adc, 0x05, 0x00);                           // lrck_l (256)
    wr_dev(s_adc, 0x07, 0x20);                           // OSR = 32
    // Enable MIC1..4 PGA + ADC
    wr_dev(s_adc, 0x43, 0x1C);                           // MIC1 gain (+24 dB)
    wr_dev(s_adc, 0x44, 0x1C);
    wr_dev(s_adc, 0x45, 0x1C);
    wr_dev(s_adc, 0x46, 0x1C);
    wr_dev(s_adc, 0x47, 0x08);
    wr_dev(s_adc, 0x48, 0x08);
    wr_dev(s_adc, 0x49, 0x08);
    wr_dev(s_adc, 0x4A, 0x08);
    wr_dev(s_adc, 0x4B, 0x0F);
    wr_dev(s_adc, 0x4C, 0x0F);
    wr_dev(s_adc, 0x01, 0x7F);                           // enable all clocks
    wr_dev(s_adc, 0x06, 0x00);                           // power up ADC
    uint8_t id1 = rd_dev(s_adc, 0x3D), id2 = rd_dev(s_adc, 0x3E);
    ESP_LOGI(TAG, "ES7210 chip id %02X%02X", id1, id2);
    return ESP_OK;
}

void board_amp(bool on)
{
    gpio_set_level(BOARD_AMP_EN, on ? 1 : 0);  // TP-01 PA_PIN is active HIGH
    s_amp = on;
}

void board_audio_set_volume(int pct)
{
    if (pct > 100) pct = 100;
    uint8_t reg = 0;
    if (pct > 0) {
        float db = 20.0f * log10f(VOLUME_CAP_PCT / 100.0f) - 30.0f * (1.0f - pct / 100.0f);
        int r = (int)lroundf(0xBF + 2.0f * db);
        reg = (uint8_t)(r < 1 ? 1 : r > 0xBF ? 0xBF : r);
    }
    if (s_dac) wr_dev(s_dac, 0x32, reg);
}

void board_audio_set_mic_gain(int step)
{
    if (step < 0) step = 0;
    if (step > 7) step = 7;
    // Map 0..7 -> ES7210 PGA gain register (0x10..0x1E)
    uint8_t val = (uint8_t)(0x10 + step * 2);
    if (s_adc) wr_dev(s_adc, 0x43, val);
}

esp_err_t board_audio_init(void)
{
    gpio_config_t amp = {.pin_bit_mask = 1ULL << BOARD_AMP_EN, .mode = GPIO_MODE_OUTPUT};
    gpio_config(&amp);
    board_amp(false);

    ESP_RETURN_ON_ERROR_LOG(board_i2c_init());

    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    cc.dma_desc_num = DMA_DESC;
    cc.dma_frame_num = DMA_FRAMES;
    cc.auto_clear = true;
    ESP_RETURN_ON_ERROR_LOG(i2s_new_channel(&cc, &s_tx, &s_rx));

    i2s_std_config_t std_tx = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(BOARD_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = BOARD_I2S_MCLK,
            .bclk = BOARD_I2S_BCLK,
            .ws = BOARD_I2S_LRCK,
            .dout = BOARD_I2S_DOUT,
            .din = I2S_GPIO_UNUSED,
        },
    };
    std_tx.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    std_tx.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
    ESP_RETURN_ON_ERROR_LOG(i2s_channel_init_std_mode(s_tx, &std_tx));

    i2s_tdm_config_t tdm_rx = {
        .clk_cfg = {
            .sample_rate_hz = BOARD_AUDIO_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            .bclk_div = 8,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = (i2s_tdm_slot_mask_t)(I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
            .ws_width = I2S_TDM_AUTO_WS_WIDTH,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false,
            .skip_mask = false,
            .total_slot = I2S_TDM_AUTO_SLOT_NUM,
        },
        .gpio_cfg = {
            .mclk = BOARD_I2S_MCLK,
            .bclk = BOARD_I2S_BCLK,
            .ws = BOARD_I2S_LRCK,
            .dout = I2S_GPIO_UNUSED,
            .din = BOARD_I2S_DIN,
        },
    };
    ESP_RETURN_ON_ERROR_LOG(i2s_channel_init_tdm_mode(s_rx, &tdm_rx));

    // MCLK must run before codecs are configured.
    ESP_RETURN_ON_ERROR_LOG(i2s_channel_enable(s_tx));
    ESP_RETURN_ON_ERROR_LOG(i2s_channel_enable(s_rx));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR_LOG(es8311_init());
    ESP_RETURN_ON_ERROR_LOG(es7210_init());
    board_audio_set_volume(70);
    ESP_LOGI(TAG, "TP-01 audio ready: %d Hz, ES8311+ES7210, MCLK 256fs", BOARD_AUDIO_RATE);
    return ESP_OK;
}

esp_err_t board_mic_start(void)
{
    board_amp(false);
    int16_t junk[DMA_FRAMES];
    while (board_mic_read(junk, DMA_FRAMES, 0) > 0) {}
    for (int i = 0; i < 4; i++) board_mic_read(junk, DMA_FRAMES, 100);
    return ESP_OK;
}

void board_mic_stop(void) {}

size_t board_mic_read(int16_t *dst, size_t n, uint32_t timeout_ms)
{
    size_t total = 0;
    int16_t tdm[128 * 4];
    while (total < n) {
        size_t chunk = n - total;
        if (chunk > 128) chunk = 128;
        size_t got_bytes = 0;
        esp_err_t e = i2s_channel_read(s_rx, tdm, chunk * 4 * sizeof(int16_t), &got_bytes, timeout_ms);
        size_t frames = got_bytes / (4 * sizeof(int16_t));
        for (size_t i = 0; i < frames; i++) {
            dst[total + i] = tdm[i * 4];  // slot 0 = MIC1
        }
        total += frames;
        if (e != ESP_OK || frames < chunk) break;
    }
    return total;
}

esp_err_t board_spk_start(void)
{
    if (!s_amp) {
        board_amp(true);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return ESP_OK;
}

size_t board_spk_write(const int16_t *src, size_t n, uint32_t timeout_ms)
{
    size_t done = 0;
    i2s_channel_write(s_tx, src, n * sizeof(int16_t), &done, timeout_ms);
    return done / sizeof(int16_t);
}

void board_spk_stop(void)
{
    static const int16_t zeros[DMA_FRAMES] = {0};
    for (int i = 0; i < DMA_DESC + 2; i++) board_spk_write(zeros, DMA_FRAMES, 100);
    board_amp(false);
}
