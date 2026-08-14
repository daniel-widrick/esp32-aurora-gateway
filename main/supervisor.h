/*
 * Liveness supervisor.
 *
 * OTA rollback only guards the first few minutes after an update. Once an
 * image is confirmed there is nothing watching it, and ESP-IDF's task watchdog
 * is configured to warn rather than reboot - so a deadlocked task on a board
 * with no USB is unrecoverable. This closes that.
 *
 * Two independent failure modes are covered:
 *
 *  - A worker task stops making progress (deadlock, starvation). Tasks call
 *    supervisor_heartbeat() each loop; if one goes quiet, reboot.
 *  - WiFi is gone long enough that the driver is probably wedged rather than
 *    the AP merely being down. Reboot to clear it. This is deliberately
 *    patient: a power cut that takes the AP out for 20 minutes should not
 *    cause a reboot loop, it should just reconnect when the AP returns.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SUPERVISOR_TASK_POLLER = 0,
    SUPERVISOR_TASK_COUNT
} supervisor_task_t;

void supervisor_start(void);

/* Called by a watched task each time round its loop. */
void supervisor_heartbeat(supervisor_task_t which);

/* Seconds since a watched task last reported, or -1 if it never has. */
double supervisor_heartbeat_age_s(supervisor_task_t which);

/* Seconds WiFi has been continuously down, 0 when connected. */
double supervisor_wifi_down_s(void);
