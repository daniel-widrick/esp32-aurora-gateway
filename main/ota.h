/*
 * Over-the-air firmware update, with rollback protection.
 *
 * The board lives at the furnace with no USB, so this is the only way to
 * change firmware. A new image boots in PENDING_VERIFY and is only marked
 * valid once WiFi and the API are actually working - otherwise the bootloader
 * reverts to the previous slot on the next reset.
 */

#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "esp_http_server.h"

/* POST a raw .bin here to update. */
esp_err_t ota_post_handler(httpd_req_t *req);

/* GET: running slot, image state, and whether a rollback is pending. */
esp_err_t ota_status_handler(httpd_req_t *req);

/* Call once the system is demonstrably healthy. Cancels the pending rollback
 * so the freshly-flashed image becomes permanent. Safe to call repeatedly. */
void ota_mark_valid(void);

/* True if this boot is a new image still awaiting confirmation. */
bool ota_pending_verify(void);

/*
 * Start the confirm-or-reboot watchdog. Only meaningful on a pending-verify
 * boot: if the image has not marked itself valid within `timeout_ms`, reboot
 * so the bootloader can roll back.
 *
 * Without this, rollback protection has a hole. The bootloader only reverts on
 * a RESET, so a bad image that hangs rather than crashes - one that boots fine
 * but never reaches WiFi - would sit there forever, unreachable and never
 * rolled back. Crashing images are covered automatically; hung ones are not.
 */
void ota_start_rollback_watchdog(int timeout_ms);
