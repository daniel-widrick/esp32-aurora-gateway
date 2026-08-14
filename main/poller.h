/*
 * Background Modbus poller with a thread-safe cache.
 *
 * The RS-485 bus is strictly one transaction at a time, and an HTTP handler
 * must never be what holds that lock while a client dawdles. So a single task
 * owns polling, publishes into a cache, and the API serves from the cache.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint32_t polls_ok;
    uint32_t polls_failed;
    int64_t last_success_us;   /* esp_timer_get_time() at last good poll */
    bool ever_succeeded;
} poller_stats_t;

esp_err_t poller_start(void);

/*
 * Polling can be suspended so the board sits electrically silent on the bus -
 * it never asserts DE, never transmits. Intended for first contact with the
 * real furnace, where the AWL is already an active master: plug in mute,
 * confirm nothing is disturbed, then enable. The setting persists in NVS, so
 * you can set it before deployment and it survives the trip.
 */
void poller_set_enabled(bool enabled);
bool poller_is_enabled(void);

/* Look up a cached register. Returns false if it isn't in the poll set or
 * hasn't been read successfully yet. */
bool poller_get(uint16_t reg, uint16_t *out);

void poller_get_stats(poller_stats_t *out);

/* Seconds since the last successful poll, or -1 if there has never been one. */
double poller_age_s(void);
