#include "auth.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "secrets.h"

static const char *TAG = "auth";

#define BEARER_PREFIX "Bearer "
#define MAX_HEADER 256

/*
 * Constant-time comparison.
 *
 * strcmp/memcmp return as soon as they hit a difference, so how long they take
 * leaks how many leading bytes were correct. That's enough to recover a token
 * byte-by-byte given enough attempts. This always walks the whole length.
 *
 * `volatile` keeps the compiler from noticing the accumulator is only tested
 * at the end and "helpfully" reintroducing an early exit.
 */
static bool constant_time_equal(const char *a, const char *b, size_t len)
{
    volatile unsigned char diff = 0;
    for (size_t i = 0; i < len; i++) {
        diff |= (unsigned char)a[i] ^ (unsigned char)b[i];
    }
    return diff == 0;
}

/*
 * Auth failures are logged at most once per window, with a running count.
 *
 * Without this, an unauthenticated caller can spam rejected requests and push
 * every useful line out of the 6 KB log ring within seconds - not memory
 * exhaustion (the ring is fixed) but a cheap way to destroy the diagnostic
 * history you'd want after an incident. Throttling keeps the signal ("someone
 * is probing, 4812 times") while costing a couple of lines.
 */
#define AUTH_LOG_WINDOW_US (10 * 1000 * 1000)

static uint32_t s_rejects;
static uint32_t s_rejects_logged;
static int64_t s_last_log_us;

static void log_rejection(const char *uri, const char *why)
{
    s_rejects++;
    int64_t now = esp_timer_get_time();
    if (s_last_log_us != 0 && (now - s_last_log_us) < AUTH_LOG_WINDOW_US) {
        return;     /* counted, deliberately not logged */
    }
    uint32_t since = s_rejects - s_rejects_logged;
    s_last_log_us = now;
    s_rejects_logged = s_rejects;
    if (since > 1) {
        ESP_LOGW(TAG, "%s: %s (%" PRIu32 " rejections in the last window, "
                 "%" PRIu32 " total)", uri, why, since, s_rejects);
    } else {
        ESP_LOGW(TAG, "%s: %s (%" PRIu32 " total)", uri, why, s_rejects);
    }
}

uint32_t auth_rejection_count(void)
{
    return s_rejects;
}

static void send_401(httpd_req_t *req)
{
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_type(req, "application/json");
    /* Tells a client how to authenticate without disclosing anything. */
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Bearer realm=\"aurora-gateway\"");
    httpd_resp_send(req, "{\"error\":\"missing or invalid bearer token\"}",
                    HTTPD_RESP_USE_STRLEN);
}

bool auth_ok(httpd_req_t *req)
{
    size_t len = httpd_req_get_hdr_value_len(req, "Authorization");
    if (len == 0 || len >= MAX_HEADER) {
        log_rejection(req->uri, "no Authorization header");
        send_401(req);
        return false;
    }

    char header[MAX_HEADER];
    if (httpd_req_get_hdr_value_str(req, "Authorization", header, sizeof(header)) != ESP_OK) {
        send_401(req);
        return false;
    }

    if (strncmp(header, BEARER_PREFIX, strlen(BEARER_PREFIX)) != 0) {
        log_rejection(req->uri, "Authorization is not a Bearer token");
        send_401(req);
        return false;
    }

    const char *token = header + strlen(BEARER_PREFIX);
    size_t token_len = strlen(token);
    size_t key_len = strlen(API_KEY);

    /* Compare the length first and bail out - lengths are not secret, and this
     * keeps the constant-time compare operating on equal-length buffers. */
    if (token_len != key_len || !constant_time_equal(token, API_KEY, key_len)) {
        /* Never log the supplied token: a near-miss in the log is a gift. */
        log_rejection(req->uri, "invalid bearer token rejected");
        send_401(req);
        return false;
    }

    return true;
}
