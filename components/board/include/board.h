// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: Copyright (C) 2026 iamankushpandit <https://github.com/iamankushpandit>
//
// Part of Ivy AI -- https://github.com/iamankushpandit/esp_ai
// Free software under GPL-3.0-or-later, with the Espressif SDK linking
// exception in LICENSE.exception. Reusing any part of this file, in any
// work, must keep this notice, credit iamankushpandit as the author,
// and stay under the same licence with corresponding source offered.
// See LICENSE, LICENSE.exception, NOTICE.md and THIRD_PARTY.md.

// TP-01 (ostb-3st) board support (native ESP-IDF v6.1).
// Pin map derived from fw-kira (ostb-3st/config.h):
//   NV3023 296x240 landscape SPI3 LCD, CST816 touch (0x15),
//   ES8311 DAC (0x18) + ES7210 ADC (0x40), PA GPIO 4 (active HIGH),
//   3 buttons (BOOT=0, VOL_UP=40, VOL_DOWN=39),
//   Battery ADC2_CH6 (GPIO 17), Charge status GPIO 47 (active LOW),
//   Internal 16MB Flash Wear-Levelling FATFS mounted at /sd.
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---- Pins (TP-01 / ostb-3st)
#define BOARD_LCD_MOSI   10
#define BOARD_LCD_SCLK   9
#define BOARD_LCD_CS     14
#define BOARD_LCD_DC     8
#define BOARD_LCD_RST    18
#define BOARD_LCD_BL     13   // active HIGH (PWM)

#define BOARD_I2C_SDA    12
#define BOARD_I2C_SCL    11
#define BOARD_TOUCH_ADDR 0x15 // CST816 capacitive touch (polled, no INT/RST)

#define BOARD_I2S_MCLK   5
#define BOARD_I2S_BCLK   15
#define BOARD_I2S_LRCK   16
#define BOARD_I2S_DOUT   6    // ESP -> ES8311 (speaker)
#define BOARD_I2S_DIN    7    // ES7210 -> ESP (mic)
#define BOARD_AMP_EN     4    // PA enable, active HIGH

#define BOARD_BOOT_BTN     0  // center power/BOOT button, active LOW
#define BOARD_VOL_UP_BTN   40 // volume up, active LOW
#define BOARD_VOL_DOWN_BTN 39 // volume down, active LOW

#define BOARD_BAT_ADC_GPIO 17 // ADC2_CH6, ~2:1 divider
#define BOARD_CHG_STAT_GPIO 47 // 0 = charging, 1 = full/idle

#define BOARD_LCD_W        296
#define BOARD_LCD_H        240
#define BOARD_LCD_OFFSET_X 24
#define BOARD_LCD_OFFSET_Y 0
#define BOARD_AUDIO_RATE   16000

// ---- Shared I2C bus (touch + codecs)
esp_err_t board_i2c_init(void);

// ---- LCD (NV3023 296x240 landscape). Pixels are RGB565 in CPU order; the
// driver byte-swaps as it sends.
esp_err_t board_lcd_init(void);
void board_backlight(uint8_t pct);  // 0 = off
void board_lcd_fill(int x, int y, int w, int h, uint16_t color);
void board_lcd_blit(int x, int y, int w, int h, const uint16_t *px);

// ---- Touch (CST816)
esp_err_t board_touch_init(void);
// Returns true while touched; screen coordinates in 296x240 landscape.
bool board_touch_read(int *x, int *y);

// ---- Physical buttons (TP-01: BOOT, VOL+, VOL-)
void board_buttons_init(void);
bool board_btn_boot(void);
bool board_btn_vol_up(void);
bool board_btn_vol_down(void);

// ---- Audio (ES8311 DAC + ES7210 ADC + I2S full-duplex, 16 kHz mono s16)
esp_err_t board_audio_init(void);
void board_amp(bool on);
void board_audio_set_volume(int pct);           // 0..100, dB-mapped, capped
void board_audio_set_mic_gain(int step);        // 0..7
size_t board_mic_read(int16_t *dst, size_t n, uint32_t timeout_ms);
esp_err_t board_mic_start(void);
void board_mic_stop(void);
size_t board_spk_write(const int16_t *src, size_t n, uint32_t timeout_ms);
esp_err_t board_spk_start(void);
void board_spk_stop(void);

// ---- Storage (Internal Flash Wear-Levelling FATFS mounted at BOARD_SD_MOUNT)
#define BOARD_SD_MOUNT "/sd"
esp_err_t board_sd_mount(bool format_if_fail);
void board_sd_unmount(void);
bool board_sd_mounted(void);
esp_err_t board_sd_format(void);
void board_sd_info(void);

// ---- Battery & Charger
int board_battery_mv(void);
bool board_battery_charging(void);

#ifdef __cplusplus
}
#endif
