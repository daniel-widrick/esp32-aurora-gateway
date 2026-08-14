/*
 * Aurora RS485 -> WiFi gateway.
 *
 * Polls the Aurora ABC over RS-485 Modbus-RTU and serves it as a JSON API over
 * HTTPS. See firmware/HARDWARE_WIRING.md for the bus and pin details.
 */

#include <inttypes.h>

#include "api.h"
#include "aurora_modbus.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "logbuf.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "ota.h"
#include "poller.h"
#include "settings.h"
#include "supervisor.h"
#include "wifi.h"

static const char *TAG = "main";

#define MDNS_HOSTNAME "aurora-gateway"
#define WIFI_TIMEOUT_MS 20000

/*
 * How long a new image must run healthily before it cancels its own rollback.
 * Long on purpose: a bug that only bites after startup (a crash on the first
 * real request, a leak that takes minutes) must trip BEFORE we mark valid, or
 * rollback is already spent and the board boot-loops on the bad image.
 *
 * The rollback watchdog must outlast this, or it would revert a healthy image
 * that simply hasn't finished proving itself yet.
 */
#define CONFIRM_MIN_UPTIME_S 300
#define ROLLBACK_TIMEOUT_MS ((CONFIRM_MIN_UPTIME_S + 120) * 1000)

/*
 * Cancel the pending OTA rollback only after the image has demonstrably held
 * together: it reached WiFi + API (guaranteed by the time this task runs) AND
 * has been up CONFIRM_MIN_UPTIME_S. If polling is enabled we additionally
 * require a successful poll, so a build that talks to the bus wrongly rolls
 * back; if polling is deliberately silent we don't (there's nothing to prove).
 * Deliberately NOT gated on WiFi staying perfect - a reachable "bus degraded"
 * board is a good outcome, per the write-path notes.
 */
static void confirm_task(void *arg)
{
    if (!ota_pending_verify()) {
        vTaskDelete(NULL);   /* already-valid image; nothing to confirm */
        return;
    }
    ESP_LOGW(TAG, "new image on trial - confirming after %ds of sustained health",
             CONFIRM_MIN_UPTIME_S);

    bool need_poll = settings_polling_enabled();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        double uptime = esp_timer_get_time() / 1e6;
        if (uptime < CONFIRM_MIN_UPTIME_S) {
            continue;
        }
        if (need_poll) {
            poller_stats_t st;
            poller_get_stats(&st);
            if (!st.ever_succeeded) {
                ESP_LOGW(TAG, "uptime ok but no successful poll yet - holding confirmation");
                continue;
            }
        }
        break;
    }
    ota_mark_valid();
    ESP_LOGI(TAG, "image confirmed healthy");
    vTaskDelete(NULL);
}

static void start_mdns(void)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        return;
    }
    /* Matches the cert SAN, so hostname verification can actually succeed
     * instead of forcing every client to disable it. */
    mdns_hostname_set(MDNS_HOSTNAME);
    mdns_instance_name_set("WaterFurnace Aurora Gateway");
    mdns_service_add(NULL, "_https", "_tcp", 443, NULL, 0);
    ESP_LOGI(TAG, "mDNS up as %s.local", MDNS_HOSTNAME);
}

void app_main(void)
{
    /* Before anything else, so early failures are visible over the API once
     * there is no console to read them from. */
    logbuf_init();
    ESP_LOGI(TAG, "Aurora RS485 -> WiFi gateway");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        /* Degrade, don't abort: aborting here boot-loops identically in BOTH
         * OTA slots, and rollback can't help because the other image has the
         * same code. A reachable gateway that forgot its settings (safe
         * defaults: writes off, polling silent) beats a bricked one. */
        ESP_LOGE(TAG, "nvs_flash_init failed: %s - running with safe defaults",
                 esp_err_to_name(err));
    }

    /* Load persisted flags BEFORE poller_start(), so a board configured silent
     * stays silent from the first poll rather than transmitting until WiFi and
     * api_start() eventually run. */
    settings_load();

    /* Bring the bus up first: if WiFi is down the gateway is useless, but if
     * the bus is down we still want the API answering with a clear "degraded"
     * rather than failing to boot. */
    ESP_ERROR_CHECK(aurora_modbus_init());
    ESP_ERROR_CHECK(poller_start());

    /* Supervisor and rollback watchdog BOTH armed before the WiFi wait - that
     * wait is exactly where a bad image hangs, and if the supervisor only
     * started after it, a board that never gets WiFi would have no watchdog at
     * all in the one state where it most needs one. */
    supervisor_start();
    ota_start_rollback_watchdog(ROLLBACK_TIMEOUT_MS);

    if (wifi_connect(WIFI_TIMEOUT_MS) != ESP_OK) {
        ESP_LOGW(TAG, "no WiFi yet - continuing; the API starts once we have an IP");
        while (!wifi_is_connected()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    start_mdns();

    /* Retry rather than abort: httpd_ssl_start is heap-sensitive, and aborting
     * boot-loops both slots. Keep trying so a transient shortage recovers. */
    while (api_start() != ESP_OK) {
        ESP_LOGE(TAG, "api_start failed - retrying in 30s");
        vTaskDelay(pdMS_TO_TICKS(30000));
    }

    /*
     * OTA confirmation is deferred to a health-gated task (see confirm_task):
     * marking the image valid the instant the API binds means a bug that
     * crashes at t+40s has already cancelled its own rollback -> boot loop.
     * The task waits for sustained uptime AND a completed poll cycle (or, if
     * the bus is deliberately silent, just sustained uptime) before confirming.
     */
    xTaskCreate(confirm_task, "ota_confirm", 3072, NULL, 4, NULL);

    ESP_LOGI(TAG, "ready: https://%s/api/status", wifi_ip_str());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        poller_stats_t st;
        poller_get_stats(&st);
        ESP_LOGI(TAG, "alive: ip=%s rssi=%d polls ok=%" PRIu32 " failed=%" PRIu32,
                 wifi_ip_str(), wifi_rssi(), st.polls_ok, st.polls_failed);
    }
}
