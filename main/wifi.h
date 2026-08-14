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
