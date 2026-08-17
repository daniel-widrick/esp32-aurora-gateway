#include "wifi.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "secrets.h"

static const char *TAG = "wifi";

#define GOT_IP_BIT BIT0
#define FAILED_BIT BIT1
#define MAX_FAST_RETRIES 8
#define SLOW_RETRY_US (5 * 1000 * 1000)

static EventGroupHandle_t s_events;
static int s_retries;
static bool s_connected;
static char s_ip[16] = "0.0.0.0";
static esp_timer_handle_t s_retry_timer;
static bool s_debug_logging;

static void retry_timer_cb(void *arg)
{
    ESP_LOGI(TAG, "retrying association");
    esp_wifi_connect();
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        strcpy(s_ip, "0.0.0.0");
        /*
         * Log the reason code. Without it a disconnect loop is undiagnosable -
         * "disconnected, retry 3/8" looks identical whether the PSK is wrong
         * (204 HANDSHAKE_TIMEOUT / 15 4WAY_HANDSHAKE_TIMEOUT), the AP is absent
         * (201 NO_AP_FOUND), or the AP is actively refusing us (202 AUTH_FAIL).
         * Those need completely different fixes, and guessing between them
         * costs a flash cycle each time.
         */
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)data;
        uint8_t reason = d ? d->reason : 0;
        /* Retry hard at first so boot is quick, then back off to a slow
         * forever-retry - a gateway that gives up on a flaky AP is useless. */
        if (s_retries < MAX_FAST_RETRIES) {
            s_retries++;
            esp_wifi_connect();
            ESP_LOGW(TAG, "disconnected (reason %u), retry %d/%d",
                     reason, s_retries, MAX_FAST_RETRIES);
        } else {
            ESP_LOGW(TAG, "disconnected (reason %u)", reason);
            xEventGroupSetBits(s_events, FAILED_BIT);
            /* Defer via a timer rather than sleeping here. This handler runs on
             * the shared event-loop task; blocking it for 5 s per disconnect
             * stalls every other event in the system, and a flapping AP would
             * keep it stalled indefinitely. */
            esp_timer_start_once(s_retry_timer, SLOW_RETRY_US);
            ESP_LOGW(TAG, "disconnected, slow retry in %d s", SLOW_RETRY_US / 1000000);
        }
        return;
    }

    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        s_retries = 0;
        s_connected = true;
        /*
         * Log which AP we actually landed on. With several APs sharing one
         * SSID, "connected" is not enough information - a weak or misbehaving
         * AP and a good one look identical in the log without the BSSID, and
         * that is the difference between a healthy link and an unreachable
         * board.
         */
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            ESP_LOGI(TAG, "got ip %s via bssid %02x:%02x:%02x:%02x:%02x:%02x "
                     "ch %u rssi %d", s_ip,
                     ap.bssid[0], ap.bssid[1], ap.bssid[2],
                     ap.bssid[3], ap.bssid[4], ap.bssid[5],
                     ap.primary, ap.rssi);
        } else {
            ESP_LOGI(TAG, "got ip %s", s_ip);
        }
        xEventGroupSetBits(s_events, GOT_IP_BIT);
    }
}

esp_err_t wifi_connect(int timeout_ms)
{
    s_events = xEventGroupCreate();

    const esp_timer_create_args_t targs = {
        .callback = retry_timer_cb,
        .name = "wifi_retry",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_retry_timer));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    /* esp_wifi_init() sets the driver's log level from CONFIG_LOG_MAXIMUM_LEVEL,
     * which we raise to VERBOSE so the block-ack logging is compiled in. Undo
     * that immediately - verbose from boot would bury everything else. It is
     * re-armed on demand via wifi_set_debug_logging(). */
    esp_wifi_internal_set_log_level(WIFI_LOG_INFO);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_wifi_event, NULL, NULL));

    wifi_config_t cfg = { 0 };
    strncpy((char *)cfg.sta.ssid, WIFI_SSID, sizeof(cfg.sta.ssid) - 1);
    strncpy((char *)cfg.sta.password, WIFI_PASS, sizeof(cfg.sta.password) - 1);
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    /*
     * Survey every channel and take the strongest AP. Safe now that A-MPDU is
     * disabled (see sdkconfig.defaults) - before that, "strongest" reliably
     * chose an AP this chip could not exchange unicast with, and the board was
     * pinned to one specific BSSID to avoid it. The pin is gone: any AP on the
     * SSID works, so there is no reason to forbid roaming or to depend on one
     * AP staying powered.
     */
    cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* SSID deliberately not logged - this ends up in shared transcripts. */
    ESP_LOGI(TAG, "connecting to configured SSID (%u chars)",
             (unsigned)strlen(WIFI_SSID));

    EventBits_t bits = xEventGroupWaitBits(s_events, GOT_IP_BIT | FAILED_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(timeout_ms));
    if (bits & GOT_IP_BIT) {
        return ESP_OK;
    }
    ESP_LOGW(TAG, "no IP after %d ms; retries continue in the background", timeout_ms);
    return ESP_ERR_TIMEOUT;
}

bool wifi_is_connected(void) { return s_connected; }

const char *wifi_ip_str(void) { return s_ip; }

void wifi_force_disconnect(void)
{
    ESP_LOGW(TAG, "forced disconnect (diagnostic)");
    /* Reset the counter so this exercises the fast-retry path, which is what a
     * real AP blip looks like. */
    s_retries = 0;
    esp_wifi_disconnect();
}

void wifi_set_debug_logging(bool enable)
{
    /*
     * CONFIG_LOG_MAXIMUM_LEVEL is VERBOSE so these statements exist in the
     * build, but esp_wifi_init() then sets the driver to that same level - i.e.
     * fully verbose from boot. wifi_connect() knocks it back to INFO
     * immediately; this is the only way to get it back up.
     */
    esp_err_t err = esp_wifi_internal_set_log_level(enable ? WIFI_LOG_VERBOSE
                                                          : WIFI_LOG_INFO);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "could not set driver log level: %s", esp_err_to_name(err));
        return;
    }
    if (enable) {
        /* Everything, not just the CONN/SCAN submodules - the block-ack
         * exchange is not attributed to any of the named submodules. */
        esp_wifi_internal_set_log_mod(WIFI_LOG_MODULE_ALL, WIFI_LOG_SUBMODULE_ALL, true);
    }
    s_debug_logging = enable;
    ESP_LOGW(TAG, "driver debug logging %s%s", enable ? "ENABLED" : "off",
             enable ? " - fills the 6 KB log ring fast, turn it off when done" : "");
}

bool wifi_debug_logging(void) { return s_debug_logging; }

int wifi_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return ap.rssi;
    }
    return 0;
}
