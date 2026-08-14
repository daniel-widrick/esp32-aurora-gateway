/*
 * Bearer-token authentication for state-changing endpoints.
 *
 * The key is baked in at build time from the repo-root .apikey file (see
 * tools/gen_secrets.py). It is only meaningful because this API is TLS-only:
 * a bearer token over plain HTTP is readable by anyone on the path, so the
 * cert and the key are one mechanism, not two.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_http_server.h"

/*
 * Returns true if the request carries a valid token.
 *
 * If it does not, a 401 has ALREADY been sent, and the caller MUST return
 * ESP_FAIL so esp_http_server closes the socket:
 *
 *     if (!auth_ok(req)) return ESP_FAIL;
 *
 * Returning ESP_OK instead would leave the socket open and send IDF into its
 * body-purge loop, which a slowloris (huge Content-Length, one byte every few
 * seconds) rides to wedge the single server task indefinitely. Closing the
 * socket ends that. Same reasoning applies to oversize-body rejections.
 */
bool auth_ok(httpd_req_t *req);

/* Total rejected requests since boot. Rejections are logged at most once per
 * 10 s window (with a count) so probing can't scrub the log ring, but every
 * one is counted here and surfaced on /api/health. */
uint32_t auth_rejection_count(void);
