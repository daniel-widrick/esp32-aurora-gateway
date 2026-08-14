#include "supervisor.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi.h"

static const char *TAG = "supervisor";

#define CHECK_INTERVAL_MS 5000

/* The poll task loops at least every 2 s (500 ms when polling is suspended),
 * so a minute of silence means it is genuinely stuck, not merely slow. */
#define POLLER_STALL_S 60.0

/*
 * How long WiFi may stay down before we assume the driver is wedged rather
 * than the AP being off. Long on purpose: a household power cut easily takes
 * an AP out for 20+ minutes, and rebooting through that achieves nothing. What
 * this catches is the case where the AP came back and we did not.
 */
#define WIFI_DOWN_REBOOT_S 1800.0

static int64_t s_heartbeat_us[SUPERVISOR_TASK_COUNT];
static int64_t s_wifi_down_since_us;
static bool s_warned_wifi;

void supervisor_heartbeat(supervisor_task_t which)
{
    if (which < SUPERVISOR_TASK_COUNT) {
        s_heartbeat_us[which] = esp_timer_get_time();
    }
}

double supervisor_heartbeat_age_s(supervisor_task_t which)
{
    if (which >= SUPERVISOR_TASK_COUNT || s_heartbeat_us[which] == 0) {
        return -1.0;
    }
    return (double)(esp_timer_get_time() - s_heartbeat_us[which]) / 1e6;
}

double supervisor_wifi_down_s(void)
{
    if (wifi_is_connected() || s_wifi_down_since_us == 0) {
        return 0.0;
    }
    return (double)(esp_timer_get_time() - s_wifi_down_since_us) / 1e6;
}

static void reboot(const char *why)
{
    ESP_LOGE(TAG, "REBOOTING: %s", why);
    vTaskDelay(pdMS_TO_TICKS(250));   /* let the log line reach the buffer */
    esp_restart();
}

static void supervisor_task(void *arg)
{
    /* Give the system a moment to come up before judging it. */
    vTaskDelay(pdMS_TO_TICKS(15000));

    while (1) {
        for (int i = 0; i < SUPERVISOR_TASK_COUNT; i++) {
            double age = supervisor_heartbeat_age_s(i);
            if (age > POLLER_STALL_S) {
                char why[96];
                snprintf(why, sizeof(why),
                         "watched task %d silent for %.0fs (deadlock?)", i, age);
                reboot(why);
            }
        }

        if (wifi_is_connected()) {
            if (s_wifi_down_since_us != 0) {
                ESP_LOGI(TAG, "WiFi recovered after %.0fs",
                         (double)(esp_timer_get_time() - s_wifi_down_since_us) / 1e6);
            }
            s_wifi_down_since_us = 0;
            s_warned_wifi = false;
        } else {
            if (s_wifi_down_since_us == 0) {
                s_wifi_down_since_us = esp_timer_get_time();
            }
            double down = supervisor_wifi_down_s();
            if (down > WIFI_DOWN_REBOOT_S) {
                reboot("WiFi down too long; assuming a wedged driver");
            }
            if (!s_warned_wifi && down > WIFI_DOWN_REBOOT_S / 2) {
                ESP_LOGW(TAG, "WiFi down %.0fs; will reboot at %.0fs",
                         down, WIFI_DOWN_REBOOT_S);
                s_warned_wifi = true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
    }
}

void supervisor_start(void)
{
    ESP_LOGI(TAG, "liveness supervisor: task stall %.0fs, wifi-down reboot %.0fs",
             POLLER_STALL_S, WIFI_DOWN_REBOOT_S);
    xTaskCreate(supervisor_task, "supervisor", 3072, NULL, 6, NULL);
}
