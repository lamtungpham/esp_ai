// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: Copyright (C) 2026 iamankushpandit <https://github.com/iamankushpandit>
//
// Part of Ivy AI -- https://github.com/iamankushpandit/esp_ai
// Free software under GPL-3.0-or-later, with the Espressif SDK linking
// exception in LICENSE.exception. Reusing any part of this file, in any
// work, must keep this notice, credit iamankushpandit as the author,
// and stay under the same licence with corresponding source offered.
// See LICENSE, LICENSE.exception, NOTICE.md and THIRD_PARTY.md.

// TP-01 (ostb-3st) battery ADC on GPIO 17 (ADC2_CH6, ~2:1 divider) and
// hardware charge status pin on GPIO 47 (0 = charging, 1 = full/idle).
#include "board.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static bool s_chg_init;

static void chg_init(void)
{
    if (s_chg_init) return;
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BOARD_CHG_STAT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);
    s_chg_init = true;
}

bool board_battery_charging(void)
{
    chg_init();
    return gpio_get_level(BOARD_CHG_STAT_GPIO) == 0;
}

int board_battery_mv(void)
{
    chg_init();
    if (!s_adc) {
        adc_oneshot_unit_init_cfg_t u = {.unit_id = BOARD_BAT_ADC_UNIT};
        if (adc_oneshot_new_unit(&u, &s_adc) != ESP_OK) return -1;
        adc_oneshot_chan_cfg_t c = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
        adc_oneshot_config_channel(s_adc, BOARD_BAT_ADC_CHAN, &c);
        adc_cali_curve_fitting_config_t cc = {
            .unit_id = BOARD_BAT_ADC_UNIT,
            .chan = BOARD_BAT_ADC_CHAN,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        adc_cali_create_scheme_curve_fitting(&cc, &s_cali);
    }
    int raw = 0, mv = 0, sum = 0, ok = 0;
    for (int i = 0; i < 8; i++) {
        // ADC2 is shared with Wi-Fi on ESP32-S3; skip sample if busy.
        if (adc_oneshot_read(s_adc, BOARD_BAT_ADC_CHAN, &raw) != ESP_OK) continue;
        if (s_cali) adc_cali_raw_to_voltage(s_cali, raw, &mv);
        else mv = raw * 3300 / 4095;
        sum += mv;
        ok++;
    }
    if (ok == 0) return -1;
    return (sum / ok) * 2;  // ~2:1 divider
}
