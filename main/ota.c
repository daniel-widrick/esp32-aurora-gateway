#include "ota.h"

#include <string.h>

#include "auth.h"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ota";

#define OTA_CHUNK 1024
/* Abort an OTA that makes no progress for this long - bounds the half-open
 * socket wedge. Generous enough for a slow but live uploader. */
#define OTA_STALL_S 30

static bool s_marked_valid;

bool ota_pending_verify(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) != ESP_OK) {
        return false;
    }
    return state == ESP_OTA_IMG_PENDING_VERIFY;
}

void ota_mark_valid(void)
{
    if (s_marked_valid) {
        return;
    }
    if (!ota_pending_verify()) {
        s_marked_valid = true;
        return;
    }
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        s_marked_valid = true;
        ESP_LOGI(TAG, "image confirmed healthy; rollback cancelled");
    } else {
        ESP_LOGW(TAG, "failed to cancel rollback - will revert on next reset");
    }
}

static void rollback_watchdog_task(void *arg)
{
    int timeout_ms = (int)(intptr_t)arg;
    const int STEP_MS = 1000;
    int waited = 0;

    while (waited < timeout_ms) {
        if (s_marked_valid || !ota_pending_verify()) {
            ESP_LOGI(TAG, "rollback watchdog stood down after %d ms", waited);
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        waited += STEP_MS;
    }

    ESP_LOGE(TAG, "image failed to confirm within %d ms - rebooting to roll back",
             timeout_ms);
    vTaskDelay(pdMS_TO_TICKS(200));   /* let the log line flush */
    esp_restart();
}

void ota_start_rollback_watchdog(int timeout_ms)
{
    if (!ota_pending_verify()) {
        return;     /* already-valid image; nothing to guard */
    }
    ESP_LOGW(TAG, "image on trial: confirm within %d ms or roll back", timeout_ms);
    xTaskCreate(rollback_watchdog_task, "ota_wdt", 3072,
                (void *)(intptr_t)timeout_ms, 4, NULL);
}

static esp_err_t json_reply(httpd_req_t *req, const char *status, cJSON *root)
{
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!body) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    free(body);
    return err;
}

esp_err_t ota_status_handler(httpd_req_t *req)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    const esp_app_desc_t *app = esp_app_get_description();

    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    esp_ota_get_state_partition(running, &state);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "running", running ? running->label : "?");
    cJSON_AddStringToObject(root, "boot", boot ? boot->label : "?");
    cJSON_AddNumberToObject(root, "image_state", state);
    cJSON_AddBoolToObject(root, "pending_verify", state == ESP_OTA_IMG_PENDING_VERIFY);
    cJSON_AddStringToObject(root, "version", app->version);
    cJSON_AddStringToObject(root, "compiled", app->date);
    return json_reply(req, HTTPD_200, root);
}

esp_err_t ota_post_handler(httpd_req_t *req)
{
    /* Checked before a single byte is accepted - an unauthenticated caller
     * must never get as far as writing to a flash partition. */
    if (!auth_ok(req)) {
        return ESP_FAIL;   /* close the socket, don't purge the body */
    }

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (target == NULL) {
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "error", "no OTA partition available");
        return json_reply(req, "500 Internal Server Error", root);
    }

    if (req->content_len <= 0) {
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "error", "empty body; POST the raw .bin");
        return json_reply(req, "400 Bad Request", root);
    }

    ESP_LOGI(TAG, "OTA starting: %d bytes -> %s", req->content_len, target->label);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, req->content_len, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
        return json_reply(req, "500 Internal Server Error", root);
    }

    char *chunk = malloc(OTA_CHUNK);
    if (chunk == NULL) {
        esp_ota_abort(handle);
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "error", "out of memory");
        return json_reply(req, "500 Internal Server Error", root);
    }

    int remaining = req->content_len;
    int64_t last_progress_us = esp_timer_get_time();
    while (remaining > 0) {
        int want = remaining < OTA_CHUNK ? remaining : OTA_CHUNK;
        int got = httpd_req_recv(req, chunk, want);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) {
            /* A half-open socket (client slept, roamed, dropped off) times out
             * every recv_wait_timeout seconds forever. Without a bound this
             * loop wedges the single httpd task permanently - killing the OTA
             * recovery path itself, the one thing that must never hang on a
             * board with no USB. Give up after no progress for OTA_STALL_S. */
            if (esp_timer_get_time() - last_progress_us > (int64_t)OTA_STALL_S * 1000000) {
                ESP_LOGE(TAG, "OTA stalled - no data for %ds, aborting", OTA_STALL_S);
                free(chunk);
                esp_ota_abort(handle);
                cJSON *root = cJSON_CreateObject();
                cJSON_AddStringToObject(root, "error", "upload stalled; aborted");
                json_reply(req, "408 Request Timeout", root);
                return ESP_FAIL;   /* close the socket */
            }
            continue;
        }
        last_progress_us = esp_timer_get_time();
        if (got <= 0) {
            ESP_LOGE(TAG, "receive failed with %d bytes left", remaining);
            free(chunk);
            esp_ota_abort(handle);
            cJSON *root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, "error", "upload truncated");
            return json_reply(req, "400 Bad Request", root);
        }
        err = esp_ota_write(handle, chunk, got);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            free(chunk);
            esp_ota_abort(handle);
            cJSON *root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
            return json_reply(req, "500 Internal Server Error", root);
        }
        remaining -= got;
    }
    free(chunk);

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        /* ESP_ERR_OTA_VALIDATE_FAILED here means the image is corrupt or not a
         * valid app - exactly the case rollback exists to survive. */
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
        return json_reply(req, "400 Bad Request", root);
    }

    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_boot_partition failed: %s", esp_err_to_name(err));
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
        return json_reply(req, "500 Internal Server Error", root);
    }

    ESP_LOGW(TAG, "OTA written to %s; rebooting", target->label);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "written", true);
    cJSON_AddStringToObject(root, "partition", target->label);
    cJSON_AddNumberToObject(root, "bytes", req->content_len);
    cJSON_AddStringToObject(root, "note",
                            "rebooting now; the new image must reach WiFi+API or the "
                            "bootloader rolls back to the previous slot");
    esp_err_t sent = json_reply(req, HTTPD_200, root);

    /* Let the response flush before pulling the rug out. */
    vTaskDelay(pdMS_TO_TICKS(1200));
    esp_restart();
    return sent;
}
