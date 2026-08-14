# ESP32 Aurora Gateway

ESP-IDF firmware that bridges a **WaterFurnace Aurora** geothermal heat-pump
controller to your network. An ESP32-S3 speaks Modbus-RTU to the Aurora control
board over RS-485 and exposes it as a small **HTTPS JSON API** plus a read-only
web dashboard — so you can monitor loop temperatures, zone setpoints, the
variable-speed pump, and fault/lockout state, and (optionally) adjust zone
setpoints, from anything on your LAN.

It's built to live unattended at the equipment with no USB access: firmware
updates go **over the air with rollback**, a liveness supervisor reboots it out
of wedged states, and there's a remote log ring for when there's no console.

> Independent, unofficial project. Not affiliated with or endorsed by
> WaterFurnace. Interfacing with your equipment is at your own risk — see
> [Safety](#safety).

## Hardware

- **Waveshare Industrial ESP32-S3 RS485/CAN board** (onboard isolated RS-485).
  RS-485 on UART1: TX=GPIO17, RX=GPIO18, auto-direction on GPIO21.
- A WaterFurnace Aurora control board with an accessible AID/RS-485 port.

Wiring, the RJ45 pinout, and the all-important 24 VAC isolation warning are in
[`docs/HARDWARE_WIRING.md`](docs/HARDWARE_WIRING.md). **Read it before making a
cable** — mis-wiring the thermostat-bus pins can damage the control board.

## Quick start

```bash
# 1. credentials + TLS cert (both gitignored)
printf 'MY_SSID\nMY_PASSWORD\n' > .wifi
python tools/gen_secrets.py       # -> main/secrets.h, generates .apikey
./tools/gen_certs.sh              # -> main/certs/ (self-signed, 10y)

# 2. build & flash (ESP-IDF v5.x)
idf.py set-target esp32s3
idf.py -p <PORT> flash monitor
```

Full build/flash/OTA/TLS/API details: [`docs/FIRMWARE.md`](docs/FIRMWARE.md).

## API at a glance

TLS only. Reads are open; every state-changing call needs a bearer token
(`Authorization: Bearer <your .apikey>`).

| Endpoint | Auth | Purpose |
|---|---|---|
| `GET /` | — | Read-only dashboard |
| `GET /api/status` | — | Decoded furnace state (from cache) |
| `GET /api/health` | — | Liveness, WiFi, bus counters, heap |
| `GET /api/registers?start=&count=` | ✅ | Raw register read (rate-limited) |
| `POST /api/setpoint` | ✅ | Set a zone heat/cool setpoint |
| `POST /api/mode` | ✅ | Toggle writes / bus polling (persisted) |
| `POST /api/ota` | ✅ | Firmware update |
| `GET /api/logs` | ✅ | In-RAM log ring |

Two safety defaults, both persisted in NVS: **writes start disabled** (the
gateway observes before it acts), and **bus polling can be set silent** so the
board sits electrically quiet on a bus it may share with another master.

## Simulator

[`simulator/`](simulator/) is a Python Modbus-RTU slave that impersonates the
Aurora, so you can build, flash, and exercise the firmware — including a
scripted freeze-fault lockout — with **no heat pump attached**. Wire the ESP32's
RS-485 A/B to a USB-RS485 adapter and:

```bash
python -m aurora_sim --port /dev/ttyUSB0 --scenario cooling
python simulator/test_aurora_sim.py     # 60+ behavioural assertions
```

See [`simulator/aurora_sim/README.md`](simulator/aurora_sim/README.md).

## Safety

- The RS-485 side may share the bus with an existing controller. Start with
  `polling_enabled: false` and confirm nothing is disturbed before enabling.
- Setpoint writes are validated against the unit's real accepted ranges before
  they reach the bus (the controller silently ignores out-of-range values
  rather than clamping them). Writes are disabled by default regardless.
- Follow [`docs/HARDWARE_WIRING.md`](docs/HARDWARE_WIRING.md) exactly for the
  cable. The AID port carries 24 VAC on some pins; shorting them can blow a fuse
  or damage the control board.

## License

[MIT](LICENSE).
