#include "wifi.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
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
        /* Retry hard at first so boot is quick, then back off to a slow
         * forever-retry - a gateway that gives up on a flaky AP is useless. */
        if (s_retries < MAX_FAST_RETRIES) {
            s_retries++;
            esp_wifi_connect();
            ESP_LOGW(TAG, "disconnected, retry %d/%d", s_retries, MAX_FAST_RETRIES);
        } else {
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
        ESP_LOGI(TAG, "got ip %s", s_ip);
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

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_wifi_event, NULL, NULL));

    wifi_config_t cfg = { 0 };
    strncpy((char *)cfg.sta.ssid, WIFI_SSID, sizeof(cfg.sta.ssid) - 1);
    strncpy((char *)cfg.sta.password, WIFI_PASS, sizeof(cfg.sta.password) - 1);
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

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

int wifi_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return ap.rssi;
    }
    return 0;
}
