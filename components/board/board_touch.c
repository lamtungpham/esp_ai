// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: Copyright (C) 2026 iamankushpandit <https://github.com/iamankushpandit>
//
// Part of Ivy AI -- https://github.com/iamankushpandit/esp_ai
// Free software under GPL-3.0-or-later, with the Espressif SDK linking
// exception in LICENSE.exception. Reusing any part of this file, in any
// work, must keep this notice, credit iamankushpandit as the author,
// and stay under the same licence with corresponding source offered.
// See LICENSE, LICENSE.exception, NOTICE.md and THIRD_PARTY.md.

// CST816 capacitive touch on the shared I2C bus (0x15), polled (no INT/RST
// lines on TP-01 / ostb-3st), plus physical button helpers (BOOT, VOL+, VOL-).
#include "board.h"
#include "board_priv.h"
#include "board_lcd_stream.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "touch";
static i2c_master_dev_handle_t s_tp;

esp_err_t board_touch_init(void)
{
    ESP_RETURN_ON_ERROR_LOG(board_i2c_init());
    ESP_RETURN_ON_ERROR_LOG(board_i2c_add(BOARD_TOUCH_ADDR, &s_tp));

    uint8_t reg = 0xA7, id = 0;
    esp_err_t e = i2c_master_transmit_receive(s_tp, &reg, 1, &id, 1, 100);
    if (e == ESP_OK) {
        ESP_LOGI(TAG, "CST816 chip id 0x%02X", id);
    } else {
        // CST816 on ostb-3st has no RST/INT and sleeps when idle until touched.
        ESP_LOGW(TAG, "CST816 asleep at boot (%s); polling active", esp_err_to_name(e));
    }
    return ESP_OK;
}

bool board_touch_read(int *x, int *y)
{
    uint8_t reg = 0x02, b[6] = {0};
    if (!s_tp || i2c_master_transmit_receive(s_tp, &reg, 1, b, sizeof(b), 50) != ESP_OK) return false;
    int n = b[0] & 0x0F;
    if (n == 0 || n > 2) return false;
    int raw_x = ((b[1] & 0x0F) << 8) | b[2];
    int raw_y = ((b[3] & 0x0F) << 8) | b[4];
    // Panel native is 240x296; display runs 296x240 landscape (swap_xy, mirror_x, mirror_y).
    int lx = (BOARD_LCD_W - 1) - raw_y;
    int ly = (BOARD_LCD_H - 1) - raw_x;
    if (lx < 0) lx = 0;
    if (lx >= BOARD_LCD_W) lx = BOARD_LCD_W - 1;
    if (ly < 0) ly = 0;
    if (ly >= BOARD_LCD_H) ly = BOARD_LCD_H - 1;
    *x = lx;
    *y = ly;
    return true;
}

void board_buttons_init(void)
{
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << BOARD_BOOT_BTN) | (1ULL << BOARD_VOL_UP_BTN) | (1ULL << BOARD_VOL_DOWN_BTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn);
}

bool board_btn_boot(void) { return gpio_get_level(BOARD_BOOT_BTN) == 0; }
bool board_btn_vol_up(void) { return gpio_get_level(BOARD_VOL_UP_BTN) == 0; }
bool board_btn_vol_down(void) { return gpio_get_level(BOARD_VOL_DOWN_BTN) == 0; }
