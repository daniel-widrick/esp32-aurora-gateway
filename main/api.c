/*
 * HTTPS JSON API over the Aurora bus.
 *
 * Status/health reads are served from the poller's cache so they never touch
 * the bus. The two endpoints that DO drive the wire - /api/registers (reads)
 * and /api/setpoint (writes) - are both authenticated, and /api/registers is
 * additionally rate-limited, so no HTTP client can stall or flood the RS-485
 * bus it shares with the AWL. Writes go to the wire only after the range
 * checks the ABC itself will NOT perform - the controller silently ignores an
 * an out-of-range setpoint is silently ignored by the IZ2, with no Modbus
 * exception, so validating here is the only thing standing between a caller
 * and a write that looks successful and does nothing.
 */

#include "api.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "auth.h"
#include "aurora_modbus.h"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_https_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "logbuf.h"
#include "ota.h"
#include "poller.h"
#include "settings.h"
#include "supervisor.h"
#include "wifi.h"

static const char *TAG = "api";

/*
 * Write-mode and polling state live in settings.c (loaded once at boot, before
 * the poll task can transmit). This module only reads them and asks settings.c
 * to persist changes. Writes default OFF and polling defaults SILENT - the
 * board observes before it acts, and stays off a shared bus until told.
 */

extern const uint8_t servercert_start[] asm("_binary_servercert_pem_start");
extern const uint8_t servercert_end[]   asm("_binary_servercert_pem_end");
extern const uint8_t prvtkey_start[]    asm("_binary_prvtkey_pem_start");
extern const uint8_t prvtkey_end[]      asm("_binary_prvtkey_pem_end");
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/* Measured setpoint limits (measured on a real unit): cooling probed directly,
 * heating per ccutrer. Out-of-range is rejected by the IZ2, not clamped. */
#define COOL_MIN_F 54
#define COOL_MAX_F 99
#define HEAT_MIN_F 40
#define HEAT_MAX_F 90

#define ZONE_HEAT_SP_BASE 21203
#define ZONE_COOL_SP_BASE 21204
#define ZONE_WRITE_STRIDE 9
#define ZONE_READ_BASE    31007
#define ZONE_CFG_REG      31101
#define MAX_ZONES 6

#define SENTINEL_NA (-9999)

/* Minimum spacing between raw register-read transactions (the one wire-driving
 * endpoint) so it can't saturate the AWL-shared bus. */
#define REGISTERS_MIN_INTERVAL_MS 500

static esp_err_t send_json(httpd_req_t *req, cJSON *root, const char *status)
{
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (body == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    free(body);
    return err;
}

static esp_err_t send_error(httpd_req_t *req, const char *status, const char *msg)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "error", msg);
    return send_json(req, root, status);
}

/*
 * Reject and CLOSE the socket. For failures raised BEFORE the request body has
 * been read: returning ESP_OK there would send IDF into its body-purge loop,
 * which a slowloris rides to wedge the server. Returning ESP_FAIL makes
 * esp_http_server close the socket instead. Use send_error (keep-alive) only
 * after the body is fully consumed.
 */
static esp_err_t reject_close(httpd_req_t *req, const char *status, const char *msg)
{
    send_error(req, status, msg);
    return ESP_FAIL;
}

/* Add a signed tenths value, or null when the ABC reports its -999.9 sentinel. */
static void add_tenths(cJSON *obj, const char *key, uint16_t raw)
{
    int16_t sv = (int16_t)raw;
    if (sv == SENTINEL_NA) {
        cJSON_AddNullToObject(obj, key);
    } else {
        cJSON_AddNumberToObject(obj, key, sv / 10.0);
    }
}

static void add_cached_tenths(cJSON *obj, const char *key, uint16_t reg)
{
    uint16_t v;
    if (poller_get(reg, &v)) {
        add_tenths(obj, key, v);
    } else {
        cJSON_AddNullToObject(obj, key);
    }
}

/* -------------------------------------------------------------- status page */

/*
 * Read-only dashboard at /. Unauthenticated, matching the other read
 * endpoints - it renders nothing the API doesn't already serve openly, and
 * deliberately does not touch /api/logs, which is protected because the WiFi
 * driver logs the SSID there. Everything is inlined in the one file: the
 * gateway has no internet, and neither may whoever is standing at the furnace
 * with a phone.
 */
static esp_err_t root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start - 1);
}

/* ------------------------------------------------------------------ health */

static esp_err_t health_get(httpd_req_t *req)
{
    poller_stats_t st;
    poller_get_stats(&st);
    const esp_app_desc_t *app = esp_app_get_description();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status",
                            st.ever_succeeded && poller_age_s() < 15.0 ? "ok" : "degraded");
    cJSON_AddNumberToObject(root, "uptime_s", esp_timer_get_time() / 1000000.0);
    cJSON_AddStringToObject(root, "version", app->version);
    cJSON_AddStringToObject(root, "idf", app->idf_ver);

    cJSON *net = cJSON_CreateObject();
    cJSON_AddBoolToObject(net, "connected", wifi_is_connected());
    cJSON_AddStringToObject(net, "ip", wifi_ip_str());
    cJSON_AddNumberToObject(net, "rssi_dbm", wifi_rssi());
    cJSON_AddItemToObject(root, "wifi", net);

    cJSON *bus = cJSON_CreateObject();
    cJSON_AddBoolToObject(bus, "polling_enabled", poller_is_enabled());
    cJSON_AddNumberToObject(bus, "polls_ok", st.polls_ok);
    cJSON_AddNumberToObject(bus, "polls_failed", st.polls_failed);
    uint32_t crc_err, addr_mm, busy;
    aurora_bus_faults(&crc_err, &addr_mm, &busy);
    cJSON_AddNumberToObject(bus, "crc_errors", crc_err);
    cJSON_AddNumberToObject(bus, "addr_mismatch", addr_mm);
    cJSON_AddNumberToObject(bus, "bus_busy", busy);
    double age = poller_age_s();
    if (age < 0) {
        cJSON_AddNullToObject(bus, "last_poll_age_s");
    } else {
        cJSON_AddNumberToObject(bus, "last_poll_age_s", age);
    }
    cJSON_AddItemToObject(root, "bus", bus);

    cJSON *sup = cJSON_CreateObject();
    double hb = supervisor_heartbeat_age_s(SUPERVISOR_TASK_POLLER);
    if (hb < 0) {
        cJSON_AddNullToObject(sup, "poller_heartbeat_age_s");
    } else {
        cJSON_AddNumberToObject(sup, "poller_heartbeat_age_s", hb);
    }
    cJSON_AddNumberToObject(sup, "wifi_down_s", supervisor_wifi_down_s());
    cJSON_AddItemToObject(root, "supervisor", sup);

    /* Heap is the thing to watch for a slow leak: free_min is the low-water
     * mark since boot, so a downward trend shows up even between polls. */
    cJSON *mem = cJSON_CreateObject();
    cJSON_AddNumberToObject(mem, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(mem, "free_heap_min", esp_get_minimum_free_heap_size());
    cJSON_AddNumberToObject(mem, "largest_block",
                            heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    cJSON_AddItemToObject(root, "memory", mem);

    cJSON *sec = cJSON_CreateObject();
    cJSON_AddNumberToObject(sec, "auth_rejections", auth_rejection_count());
    cJSON_AddNumberToObject(sec, "log_bytes_buffered", logbuf_len());
    cJSON_AddNumberToObject(sec, "log_bytes_total", logbuf_total_bytes());
    cJSON_AddItemToObject(root, "diag", sec);

    return send_json(req, root, HTTPD_200);
}

static esp_err_t debug_wifi_drop_post(httpd_req_t *req)
{
    if (!auth_ok(req)) {
        return ESP_FAIL;   /* close the socket, don't purge the body */
    }

    wifi_force_disconnect();
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "disconnected", true);
    cJSON_AddStringToObject(root, "note",
                            "reconnect should happen automatically; poll /api/health");
    return send_json(req, root, HTTPD_200);
}

/* ------------------------------------------------------------------ status */

static esp_err_t status_get(httpd_req_t *req)
{
    uint16_t v;
    cJSON *root = cJSON_CreateObject();

    double age = poller_age_s();
    if (age < 0) {
        cJSON_Delete(root);
        return send_error(req, "503 Service Unavailable", "no successful poll yet");
    }
    cJSON_AddNumberToObject(root, "age_s", age);

    cJSON *loop = cJSON_CreateObject();
    add_cached_tenths(loop, "leaving_water_f", 1110);
    add_cached_tenths(loop, "entering_water_f", 1111);
    add_cached_tenths(loop, "leaving_air_f", 1112);
    add_cached_tenths(loop, "suction_f", 1113);
    add_cached_tenths(loop, "discharge_psi", 1115);
    add_cached_tenths(loop, "fp1_f", 19);
    add_cached_tenths(loop, "fp2_f", 20);
    add_cached_tenths(loop, "compressor_ambient_f", 3326);
    if (poller_get(325, &v)) {
        cJSON_AddNumberToObject(loop, "pump_output_pct", v);
    }
    cJSON_AddItemToObject(root, "loop", loop);

    /* reg 25: low 15 bits = stored E-code, bit 15 = lockout. */
    cJSON *fault = cJSON_CreateObject();
    if (poller_get(25, &v)) {
        cJSON_AddNumberToObject(fault, "code", v & 0x7FFF);
        cJSON_AddBoolToObject(fault, "lockout", (v & 0x8000) != 0);
    } else {
        cJSON_AddNullToObject(fault, "code");
    }
    cJSON_AddItemToObject(root, "fault", fault);

    /* Zones: temp at 31007+3z, setpoints packed across the next two words. */
    cJSON *zones = cJSON_CreateArray();
    uint16_t zcfg = 0;
    int nz = poller_get(ZONE_CFG_REG, &zcfg) ? (zcfg >> 8) & 7 : 0;
    for (int z = 0; z < nz && z < MAX_ZONES; z++) {
        uint16_t base = ZONE_READ_BASE + 3 * z, tv, hi, lo;
        if (!poller_get(base, &tv) || !poller_get(base + 1, &hi) ||
            !poller_get(base + 2, &lo)) {
            continue;
        }
        uint32_t data = ((uint32_t)hi << 16) | lo;
        cJSON *zone = cJSON_CreateObject();
        cJSON_AddNumberToObject(zone, "zone", z + 1);
        add_tenths(zone, "temp_f", tv);
        cJSON_AddNumberToObject(zone, "heat_sp_f", ((data >> 11) & 0x3F) + 36);
        cJSON_AddNumberToObject(zone, "cool_sp_f", ((data >> 17) & 0x3F) + 36);
        cJSON_AddItemToArray(zones, zone);
    }
    cJSON_AddItemToObject(root, "zones", zones);

    return send_json(req, root, HTTPD_200);
}

/* --------------------------------------------------------------- registers */

static esp_err_t registers_get(httpd_req_t *req)
{
    /*
     * Authenticated, unlike the other reads: this is the ONE read endpoint that
     * drives the RS-485 wire synchronously (not the poller cache), so an open
     * one would let a LAN attacker enumerate the whole register map AND flood a
     * bus shared with the AWL. It's a dealer/debug tool, not used by the
     * dashboard.
     */
    if (!auth_ok(req)) {
        return ESP_FAIL;
    }

    /* Rate-limit wire access so even a token-holder can't saturate the shared
     * bus: one transaction per REGISTERS_MIN_INTERVAL_MS. */
    static int64_t s_last_reg_us;
    int64_t now = esp_timer_get_time();
    if (s_last_reg_us != 0 && (now - s_last_reg_us) < REGISTERS_MIN_INTERVAL_MS * 1000) {
        return send_error(req, "429 Too Many Requests",
                          "register reads are rate-limited; retry shortly");
    }
    s_last_reg_us = now;

    char query[64];
    long start = -1, count = 1;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(query, "start", val, sizeof(val)) == ESP_OK) {
            start = strtol(val, NULL, 10);
        }
        if (httpd_query_key_value(query, "count", val, sizeof(val)) == ESP_OK) {
            count = strtol(val, NULL, 10);
        }
    }

    if (start < 0 || start > 65535) {
        return send_error(req, "400 Bad Request", "start must be 0..65535");
    }
    if (count < 1 || count > 64) {
        return send_error(req, "400 Bad Request", "count must be 1..64");
    }

    uint16_t buf[64];
    esp_err_t err = aurora_read_holding((uint16_t)start, (uint16_t)count, buf);
    if (err != ESP_OK) {
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
        uint8_t exc = aurora_last_exception();
        if (exc) {
            cJSON_AddNumberToObject(root, "modbus_exception", exc);
        }
        return send_json(req, root, "502 Bad Gateway");
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "start", start);
    cJSON *arr = cJSON_CreateArray();
    for (long i = 0; i < count; i++) {
        cJSON_AddItemToArray(arr, cJSON_CreateNumber(buf[i]));
    }
    cJSON_AddItemToObject(root, "values", arr);
    return send_json(req, root, HTTPD_200);
}

/* ---------------------------------------------------------------- setpoint */

static esp_err_t setpoint_post(httpd_req_t *req)
{
    if (!auth_ok(req)) {
        return ESP_FAIL;   /* close the socket, don't purge the body */
    }

    char body[192];
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(body)) {
        /* Body unread here - close, don't purge (slowloris defense). */
        return reject_close(req, "400 Bad Request", "body must be 1..191 bytes");
    }
    int got = 0;
    while (got < total) {
        int n = httpd_req_recv(req, body + got, total - got);
        if (n <= 0) {
            return reject_close(req, "400 Bad Request", "truncated body");
        }
        got += n;
    }
    body[got] = '\0';

    if (!settings_writes_enabled()) {
        return send_error(req, "403 Forbidden",
                          "writes are disabled; POST {\"writes_enabled\":true} to "
                          "/api/mode to enable them");
    }

    cJSON *in = cJSON_Parse(body);
    if (in == NULL) {
        return send_error(req, "400 Bad Request", "invalid JSON");
    }

    cJSON *jzone = cJSON_GetObjectItem(in, "zone");
    cJSON *jmode = cJSON_GetObjectItem(in, "mode");
    cJSON *jval = cJSON_GetObjectItem(in, "value_f");

    if (!cJSON_IsNumber(jzone) || !cJSON_IsString(jmode) || !cJSON_IsNumber(jval)) {
        cJSON_Delete(in);
        return send_error(req, "400 Bad Request",
                          "need zone (number), mode (\"heat\"|\"cool\"), value_f (number)");
    }

    int zone = jzone->valueint;
    double value_f = jval->valuedouble;
    bool cooling = strcmp(jmode->valuestring, "cool") == 0;
    bool heating = strcmp(jmode->valuestring, "heat") == 0;
    cJSON_Delete(in);

    if (!cooling && !heating) {
        return send_error(req, "400 Bad Request", "mode must be \"heat\" or \"cool\"");
    }
    if (zone < 1 || zone > MAX_ZONES) {
        return send_error(req, "400 Bad Request", "zone out of range");
    }

    int lo = cooling ? COOL_MIN_F : HEAT_MIN_F;
    int hi = cooling ? COOL_MAX_F : HEAT_MAX_F;
    if (value_f < lo || value_f > hi) {
        /* The ABC would accept this on the wire and silently not apply it. */
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "%s setpoint must be %d..%d F; the ABC would silently ignore this",
                 cooling ? "cooling" : "heating", lo, hi);
        return send_error(req, "422 Unprocessable Entity", msg);
    }

    uint16_t reg = (cooling ? ZONE_COOL_SP_BASE : ZONE_HEAT_SP_BASE)
                   + ZONE_WRITE_STRIDE * (zone - 1);
    uint16_t raw = (uint16_t)lround(value_f * 10.0);   /* F x 10 */

    esp_err_t err = aurora_write_single(reg, raw);
    if (err != ESP_OK) {
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
        uint8_t exc = aurora_last_exception();
        if (exc) {
            cJSON_AddNumberToObject(root, "modbus_exception", exc);
        }
        return send_json(req, root, "502 Bad Gateway");
    }

    ESP_LOGI(TAG, "zone %d %s setpoint -> %.1f F (reg %u = %u)",
             zone, cooling ? "cool" : "heat", value_f, reg, raw);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "written", true);
    cJSON_AddNumberToObject(root, "register", reg);
    cJSON_AddNumberToObject(root, "raw", raw);
    /* Applies on the IZ2 sync cycle, and the schedule may reassert it later. */
    cJSON_AddStringToObject(root, "note",
                            "applies on the IZ2 sync cycle (~10-20s); confirm via "
                            "/api/status, and note the schedule may reassert it");
    return send_json(req, root, HTTPD_200);
}

/* -------------------------------------------------------------- logs, mode */

static esp_err_t logs_get(httpd_req_t *req)
{
    /* A read, but protected anyway: the IDF WiFi driver logs the SSID at
     * association ("connected with <ssid>"), so an open /api/logs discloses
     * network details we otherwise take care never to emit. */
    if (!auth_ok(req)) {
        return ESP_FAIL;   /* close the socket, don't purge the body */
    }

    /* Sized to the ring exactly - it was 6300 for no reason, and every byte
     * here is permanently resident .bss. Static because it must not live on
     * the httpd task stack; safe because esp_http_server serves requests from
     * a single task. */
    static char dump[LOGBUF_SIZE + 1];
    size_t n = logbuf_dump(dump, sizeof(dump));
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, dump, n);
}

static esp_err_t mode_get(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "writes_enabled", settings_writes_enabled());
    cJSON_AddBoolToObject(root, "polling_enabled", poller_is_enabled());
    return send_json(req, root, HTTPD_200);
}

static esp_err_t mode_post(httpd_req_t *req)
{
    if (!auth_ok(req)) {
        return ESP_FAIL;   /* close the socket, don't purge the body */
    }

    char body[96];
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(body)) {
        return reject_close(req, "400 Bad Request", "body must be 1..95 bytes");
    }
    int got = 0;
    while (got < total) {
        int n = httpd_req_recv(req, body + got, total - got);
        if (n <= 0) {
            return reject_close(req, "400 Bad Request", "truncated body");
        }
        got += n;
    }
    body[got] = '\0';

    cJSON *in = cJSON_Parse(body);
    if (in == NULL) {
        return send_error(req, "400 Bad Request", "invalid JSON");
    }
    cJSON *jw = cJSON_GetObjectItem(in, "writes_enabled");
    cJSON *jp = cJSON_GetObjectItem(in, "polling_enabled");
    if (!cJSON_IsBool(jw) && !cJSON_IsBool(jp)) {
        cJSON_Delete(in);
        return send_error(req, "400 Bad Request",
                          "need writes_enabled and/or polling_enabled (bool)");
    }

    esp_err_t err = ESP_OK;
    if (cJSON_IsBool(jw)) {
        bool enable = cJSON_IsTrue(jw);
        err = settings_set_writes_enabled(enable);
        if (err == ESP_OK) {
            ESP_LOGW(TAG, "writes %s", enable ? "ENABLED" : "disabled");
        }
    }
    if (err == ESP_OK && cJSON_IsBool(jp)) {
        bool enable = cJSON_IsTrue(jp);
        err = settings_set_polling_enabled(enable);
        if (err == ESP_OK) {
            poller_set_enabled(enable);   /* apply to the running task now */
        }
    }
    cJSON_Delete(in);

    if (err != ESP_OK) {
        return send_error(req, "500 Internal Server Error", esp_err_to_name(err));
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "writes_enabled", settings_writes_enabled());
    cJSON_AddBoolToObject(root, "polling_enabled", poller_is_enabled());
    return send_json(req, root, HTTPD_200);
}

/* ------------------------------------------------------------------ server */

esp_err_t api_start(void)
{
    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
    conf.servercert = servercert_start;
    conf.servercert_len = servercert_end - servercert_start;
    conf.prvtkey_pem = prvtkey_start;
    conf.prvtkey_len = prvtkey_end - prvtkey_start;
    conf.httpd.max_uri_handlers = 12;
    conf.httpd.lru_purge_enable = true;
    conf.httpd.stack_size = 10240;
    /* Short socket timeouts bound slowloris-style attacks: a stalled or
     * dribbling connection is dropped in ~2 s instead of holding the single
     * server task for the 5 s default per recv. */
    conf.httpd.recv_wait_timeout = 2;
    conf.httpd.send_wait_timeout = 2;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_ssl_start(&server, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ssl_start failed: %s", esp_err_to_name(err));
        return err;
    }

    /*
     * Silence IDF's network-noise tags so an unauthenticated attacker cannot
     * scrub the 6 KB diagnostic ring by flooding connections. These all log at
     * WARN/ERROR on any connection, 404, 405, or failed handshake - no
     * credentials needed. Measured before: 79 of 103 buffered lines were the
     * esp_https_server handshake message alone. auth.c already throttles its
     * own rejection lines; this closes the IDF-side paths around it.
     */
    esp_log_level_set("esp_https_server", ESP_LOG_NONE);
    esp_log_level_set("httpd_uri", ESP_LOG_NONE);
    esp_log_level_set("httpd_parse", ESP_LOG_NONE);
    esp_log_level_set("httpd_txrx", ESP_LOG_NONE);
    esp_log_level_set("esp-tls", ESP_LOG_NONE);

    static const httpd_uri_t routes[] = {
        { .uri = "/",               .method = HTTP_GET,  .handler = root_get },
        { .uri = "/api/health",     .method = HTTP_GET,  .handler = health_get },
        { .uri = "/api/status",     .method = HTTP_GET,  .handler = status_get },
        { .uri = "/api/registers",  .method = HTTP_GET,  .handler = registers_get },
        { .uri = "/api/setpoint",   .method = HTTP_POST, .handler = setpoint_post },
        { .uri = "/api/logs",       .method = HTTP_GET,  .handler = logs_get },
        { .uri = "/api/mode",       .method = HTTP_GET,  .handler = mode_get },
        { .uri = "/api/mode",       .method = HTTP_POST, .handler = mode_post },
        { .uri = "/api/ota",        .method = HTTP_POST, .handler = ota_post_handler },
        { .uri = "/api/ota/status", .method = HTTP_GET,  .handler = ota_status_handler },
        { .uri = "/api/debug/wifi-drop", .method = HTTP_POST,
          .handler = debug_wifi_drop_post },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &routes[i]));
    }

    ESP_LOGI(TAG, "HTTPS API on https://%s/  (and https://aurora-gateway.local/)",
             wifi_ip_str());
    ESP_LOGW(TAG, "writes are %s", settings_writes_enabled() ? "ENABLED" : "disabled (read-only)");
    return ESP_OK;
}
