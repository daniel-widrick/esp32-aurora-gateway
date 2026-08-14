/*
 * Minimal Modbus-RTU master for the WaterFurnace Aurora ABC control board.
 *
 * Bus parameters are not configurable on purpose - they are properties of the
 * Aurora, recovered from the AWL firmware and corroborated independently.
 * See docs/HARDWARE_WIRING.md.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

/* Waveshare Industrial ESP32-S3 RS485/CAN board pin map. */
#define AURORA_UART_PORT   UART_NUM_1
#define AURORA_PIN_TXD     17
#define AURORA_PIN_RXD     18
#define AURORA_PIN_DE      21   /* transceiver direction; driven as UART RTS */

#define AURORA_BAUD        19200
#define AURORA_SLAVE_ADDR  1    /* addr 2 mirrors the same data */

/* Value the Aurora reports for a sensor that is absent or faulted: -999.9,
 * i.e. -9999 in tenths, 0xD8F1 on the wire. */
#define AURORA_SENTINEL_NA  (-9999)

esp_err_t aurora_modbus_init(void);

/* FC03. `out` receives `count` registers. Serialised internally - only one
 * transaction is ever in flight, which the AWL also enforced. */
esp_err_t aurora_read_holding(uint16_t start, uint16_t count, uint16_t *out);

/* FC06. Guarded by the setpoint validation in api.c -
 * this function does not itself validate that `reg` is safe to write. */
esp_err_t aurora_write_single(uint16_t reg, uint16_t value);

/* Last Modbus exception code seen, 0 if none. Exception 2 = illegal/read-only
 * address, which is how the ABC rejects an unmapped or non-writable register.
 * Cleared at the start of every transaction, so it always reflects the most
 * recent one. */
uint8_t aurora_last_exception(void);

/* Cumulative bus-health counters (any pointer may be NULL):
 *  crc_errors    - frames that failed CRC (noise, or a foreign frame's tail)
 *  addr_mismatch - a well-formed reply from the wrong slave address
 *  bus_busy      - transmits made without ever seeing a t3.5 idle gap (the
 *                  AWL was saturating the shared segment)
 * Rising counts mean contention on the AWL-shared bus. */
void aurora_bus_faults(uint32_t *crc_errors, uint32_t *addr_mismatch, uint32_t *bus_busy);
