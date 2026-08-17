#pragma once

#include <stdbool.h>

#include "esp_err.h"

/* Connect as a station and block until we have an IP or the timeout expires.
 * Reconnects automatically thereafter. */
esp_err_t wifi_connect(int timeout_ms);

bool wifi_is_connected(void);

/* Dotted-quad of the current IP, or "0.0.0.0". */
const char *wifi_ip_str(void);

/* Current AP signal strength in dBm, 0 if unknown. */
int wifi_rssi(void);

/* Force a disconnect. Diagnostic: the only practical way to exercise the
 * reconnect path on a deployed board without switching the AP off. Normal
 * retry/backoff then applies, so the board should come back on its own. */
void wifi_force_disconnect(void);

/*
 * Raise the WiFi driver's own log verbosity at runtime.
 *
 * The driver is a prebuilt blob whose logging is controlled separately from
 * ESP_LOG - esp_log_level_set("wifi", ...) does not reach it. This is what
 * exposes the block-ack state machine (ADDBA/DELBA), which is where the
 * aggregation fault that made this board unreachable showed itself.
 *
 * Deliberately NOT persisted: it reverts to quiet on every reboot. Left on, it
 * fills the 6 KB log ring in seconds and destroys the diagnostic history you
 * would actually want - and on a board with no USB, that is unrecoverable
 * until someone turns it off again.
 */
void wifi_set_debug_logging(bool enable);

bool wifi_debug_logging(void);
