// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: Copyright (C) 2026 iamankushpandit <https://github.com/iamankushpandit>
//
// Part of Ivy AI -- https://github.com/iamankushpandit/esp_ai
// Free software under GPL-3.0-or-later, with the Espressif SDK linking
// exception in LICENSE.exception. Reusing any part of this file, in any
// work, must keep this notice, credit iamankushpandit as the author,
// and stay under the same licence with corresponding source offered.
// See LICENSE, LICENSE.exception, NOTICE.md and THIRD_PARTY.md.

// Internal 16MB SPI Flash Wear-Levelling FATFS mounted at /sd (TP-01 has no
// physical microSD slot). Keeps /sd/... paths and tools/sd_put.py working.
#include "board.h"
#include "esp_vfs_fat.h"
#include "wear_levelling.h"
#include "esp_log.h"

static const char *TAG = "sd_flash";
static wl_handle_t s_wl = WL_INVALID_HANDLE;
static bool s_mounted;

esp_err_t board_sd_mount(bool format_if_fail)
{
    if (s_mounted) return ESP_OK;
    (void)format_if_fail;
    const esp_vfs_fat_mount_config_t mc = {
        .format_if_mount_failed = true,   // auto-format "storage" on first TP-01 boot
        .max_files = 6,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
    };
    esp_err_t e = esp_vfs_fat_spiflash_mount_rw_wl(BOARD_SD_MOUNT, "storage", &mc, &s_wl);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "Flash FATFS mount failed: %s", esp_err_to_name(e));
        s_wl = WL_INVALID_HANDLE;
        s_mounted = false;
        return e;
    }
    s_mounted = true;
    board_sd_info();
    return ESP_OK;
}

void board_sd_info(void)
{
    if (!s_mounted) return;
    uint64_t total = 0, free_b = 0;
    if (esp_vfs_fat_info(BOARD_SD_MOUNT, &total, &free_b) == ESP_OK) {
        ESP_LOGI(TAG, "Flash FATFS /sd: total %llu KB, free %llu KB",
                 total >> 10, free_b >> 10);
    }
}

esp_err_t board_sd_format(void)
{
    if (!s_mounted) return ESP_ERR_INVALID_STATE;
    ESP_LOGW(TAG, "formatting internal flash FATFS /sd...");
    esp_err_t e = esp_vfs_fat_spiflash_format_rw_wl(BOARD_SD_MOUNT, "storage");
    ESP_LOGI(TAG, "format: %s", esp_err_to_name(e));
    return e;
}

void board_sd_unmount(void)
{
    if (!s_mounted) return;
    esp_vfs_fat_spiflash_unmount_rw_wl(BOARD_SD_MOUNT, s_wl);
    s_wl = WL_INVALID_HANDLE;
    s_mounted = false;
}

bool board_sd_mounted(void) { return s_mounted; }
