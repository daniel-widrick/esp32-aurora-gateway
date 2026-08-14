#include "poller.h"

#include <string.h>

#include "aurora_modbus.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "esp_task_wdt.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "settings.h"
#include "supervisor.h"

static const char *TAG = "poller";

#define POLL_INTERVAL_MS 2000

/*
 * Register spans to poll. Contiguous runs are read in one FC03 transaction -
 * six registers in one round trip rather than six round trips.
 */
typedef struct {
    uint16_t start;
    uint16_t count;
} span_t;

static const span_t SPANS[] = {
    {   19,  2 },   /* Cooling LL / FP1, EWT / FP2                */
    {   25,  1 },   /* fault: low 15 bits = E-code, bit15 lockout */
    {  325,  1 },   /* VS pump output %                           */
    { 1110,  6 },   /* leaving/entering water, air, suction, disch */
    { 3326,  1 },   /* compressor ambient                          */
    { 31007, 6 },   /* IZ2 zone read block: Z1 and Z2              */
    { 31101, 1 },   /* zone count in bits 8-10                     */
};

#define SPAN_COUNT (sizeof(SPANS) / sizeof(SPANS[0]))

/* Flattened cache, one slot per register across all spans. */
#define CACHE_MAX 32
typedef struct {
    uint16_t reg;
    uint16_t val;
    bool valid;
} slot_t;

static slot_t s_cache[CACHE_MAX];
static size_t s_cache_len;
static SemaphoreHandle_t s_cache_lock;
static poller_stats_t s_stats;
/* Fail-safe default is SILENT: the task starts before settings are guaranteed
 * loaded, and transmitting onto a bus shared with the AWL must never be the
 * accidental default. poller_start() sets the real value from persisted
 * settings before the loop does anything. */
static volatile bool s_enabled = false;

void poller_set_enabled(bool enabled)
{
    s_enabled = enabled;
    ESP_LOGW(TAG, "polling %s", enabled ? "ENABLED" : "suspended (bus silent)");
}

bool poller_is_enabled(void)
{
    return s_enabled;
}

static void cache_init(void)
{
    s_cache_len = 0;
    for (size_t i = 0; i < SPAN_COUNT; i++) {
        for (uint16_t r = 0; r < SPANS[i].count && s_cache_len < CACHE_MAX; r++) {
            s_cache[s_cache_len].reg = SPANS[i].start + r;
            s_cache[s_cache_len].valid = false;
            s_cache_len++;
        }
    }
}

static void cache_put(uint16_t reg, uint16_t val)
{
    for (size_t i = 0; i < s_cache_len; i++) {
        if (s_cache[i].reg == reg) {
            s_cache[i].val = val;
            s_cache[i].valid = true;
            return;
        }
    }
}

bool poller_get(uint16_t reg, uint16_t *out)
{
    bool found = false;
    xSemaphoreTake(s_cache_lock, portMAX_DELAY);
    for (size_t i = 0; i < s_cache_len; i++) {
        if (s_cache[i].reg == reg && s_cache[i].valid) {
            *out = s_cache[i].val;
            found = true;
            break;
        }
    }
    xSemaphoreGive(s_cache_lock);
    return found;
}

void poller_get_stats(poller_stats_t *out)
{
    xSemaphoreTake(s_cache_lock, portMAX_DELAY);
    *out = s_stats;
    xSemaphoreGive(s_cache_lock);
}

double poller_age_s(void)
{
    poller_stats_t st;
    poller_get_stats(&st);
    if (!st.ever_succeeded) {
        return -1.0;
    }
    return (double)(esp_timer_get_time() - st.last_success_us) / 1e6;
}

/* Every SPANS[i].count must fit poll_task's buffer, or a future edit silently
 * smashes the task stack (aurora_read_holding writes count words, unchecked). */
#define POLL_BUF_REGS 8
_Static_assert(POLL_BUF_REGS <= 125, "FC03 max is 125 registers");

static void poll_task(void *arg)
{
    uint16_t buf[POLL_BUF_REGS];

    /* Subscribe to the task watchdog so a genuine stall of THIS task (not just
     * idle-CPU starvation) forces a panic-reboot. The supervisor is the
     * application-level net; this is the kernel-level one. */
    esp_task_wdt_add(NULL);

    while (1) {
        /* Report liveness before doing anything, so the supervisor can tell a
         * stuck task from a merely idle or bus-less one. */
        supervisor_heartbeat(SUPERVISOR_TASK_POLLER);
        esp_task_wdt_reset();

        if (!s_enabled) {
            /* Silent: no transmission at all, so the bus sees nothing from us. */
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        bool all_ok = true;

        for (size_t i = 0; i < SPAN_COUNT; i++) {
            /* Reset per span, not just per loop: a dead bus under HTTP mutex
             * contention makes one loop take up to ~5.7 s, which would exceed
             * the 5 s TWDT and panic a HEALTHY board. Per-span keeps the gap to
             * one transaction (~0.5 s worst case). */
            esp_task_wdt_reset();
            esp_err_t err = aurora_read_holding(SPANS[i].start, SPANS[i].count, buf);
            if (err != ESP_OK) {
                all_ok = false;
                ESP_LOGW(TAG, "span %u..%u failed: %s", SPANS[i].start,
                         SPANS[i].start + SPANS[i].count - 1, esp_err_to_name(err));
                continue;
            }
            xSemaphoreTake(s_cache_lock, portMAX_DELAY);
            for (uint16_t r = 0; r < SPANS[i].count; r++) {
                cache_put(SPANS[i].start + r, buf[r]);
            }
            xSemaphoreGive(s_cache_lock);
        }

        xSemaphoreTake(s_cache_lock, portMAX_DELAY);
        if (all_ok) {
            s_stats.polls_ok++;
            s_stats.last_success_us = esp_timer_get_time();
            s_stats.ever_succeeded = true;
        } else {
            s_stats.polls_failed++;
        }
        xSemaphoreGive(s_cache_lock);

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

esp_err_t poller_start(void)
{
    s_cache_lock = xSemaphoreCreateMutex();
    if (s_cache_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memset(&s_stats, 0, sizeof(s_stats));
    cache_init();

    /* Apply the persisted polling flag BEFORE the task can transmit. settings
     * must already be loaded (settings_load() runs earlier in app_main). */
    s_enabled = settings_polling_enabled();

    if (xTaskCreate(poll_task, "aurora_poll", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "polling %s: %u spans (%u registers) every %d ms",
             s_enabled ? "ENABLED" : "silent (persisted)",
             (unsigned)SPAN_COUNT, (unsigned)s_cache_len, POLL_INTERVAL_MS);
    return ESP_OK;
}
