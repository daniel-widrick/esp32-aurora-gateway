#include "settings.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "settings";

#define NVS_NAMESPACE "gateway"
#define KEY_WRITES "writes_en"
#define KEY_POLL "poll_en"

/*
 * Fail-safe defaults: writes OFF (observe before acting) and polling OFF (sit
 * electrically silent on a bus that already has the AWL as master). The stored
 * value turns polling on; if NVS can't be read we stay silent rather than
 * transmit blindly.
 */
static bool s_writes_enabled;
static bool s_polling_enabled;

static uint8_t get_flag(nvs_handle_t h, const char *key, uint8_t fallback)
{
    uint8_t v = fallback;
    if (nvs_get_u8(h, key, &v) != ESP_OK) {
        v = fallback;
    }
    return v;
}

void settings_load(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        /* First boot (namespace absent) is normal; anything else we note but
         * still fall back to the safe defaults. */
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "nvs_open failed: %s - using safe defaults", esp_err_to_name(err));
        }
        s_writes_enabled = false;
        s_polling_enabled = false;
        return;
    }
    s_writes_enabled = get_flag(h, KEY_WRITES, 0) != 0;
    s_polling_enabled = get_flag(h, KEY_POLL, 0) != 0;
    nvs_close(h);
    ESP_LOGI(TAG, "loaded: writes=%s polling=%s",
             s_writes_enabled ? "on" : "off", s_polling_enabled ? "on" : "off");
}

bool settings_writes_enabled(void) { return s_writes_enabled; }
bool settings_polling_enabled(void) { return s_polling_enabled; }

static esp_err_t put_flag(const char *key, bool enabled)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(h, key, enabled ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t settings_set_writes_enabled(bool enabled)
{
    esp_err_t err = put_flag(KEY_WRITES, enabled);
    if (err == ESP_OK) {
        s_writes_enabled = enabled;
    }
    return err;
}

esp_err_t settings_set_polling_enabled(bool enabled)
{
    esp_err_t err = put_flag(KEY_POLL, enabled);
    if (err == ESP_OK) {
        s_polling_enabled = enabled;
    }
    return err;
}
