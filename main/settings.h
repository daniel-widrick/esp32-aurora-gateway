/*
 * Persisted operating flags, loaded once at boot before anything that depends
 * on them.
 *
 * These were previously read inside api_start(), which runs only after the
 * (possibly unbounded) WiFi wait - so a board configured to sit silent on the
 * bus would poll anyway on every boot, and forever if WiFi never came back.
 * Loading them here, before poller_start(), makes the fail-safe default
 * actually hold.
 */

#pragma once

#include <stdbool.h>

#include "esp_err.h"

void settings_load(void);

bool settings_writes_enabled(void);
bool settings_polling_enabled(void);

/* Persist and update the in-memory copy. */
esp_err_t settings_set_writes_enabled(bool enabled);
esp_err_t settings_set_polling_enabled(bool enabled);
