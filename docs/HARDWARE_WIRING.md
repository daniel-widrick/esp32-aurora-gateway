# Hardware, wiring, and bus topology

How to wire the ESP32-S3 to a WaterFurnace Aurora control board over RS-485, and
how to share the bus with an existing controller without a fight.

Much of the RS-485 pinout and serial detail here is cross-checked against the
open-source [ccutrer/waterfurnace_aurora](https://github.com/ccutrer/waterfurnace_aurora)
project; quoted lines are from its README.

## Board and GPIO map

**Waveshare Industrial ESP32-S3 RS485/CAN board** — onboard isolated RS-485
transceiver with A/B screw terminals, so no external transceiver wiring.

| Signal | GPIO |
|---|---|
| RS-485 TX (UART1) | 17 |
| RS-485 RX (UART1) | 18 |
| RS-485 direction (auto) | 21 (driven as RTS) |

The firmware uses `uart_set_mode(UART_MODE_RS485_HALF_DUPLEX)` with RTS mapped to
GPIO21. On this board the transceiver auto-directions and IDF's RTS sense matches
it — no inversion needed. On a different board, if the driver never transmits,
that direction sense is the first thing to check.

## AID port — connector and pinout

The Aurora ABC's **AID Tool port is an RJ45 jack**, wired to **TIA-568-B**.
ccutrer's README, verbatim:

> "Connect pins 1 and 3 (white/orange and white/green for a TIA-568-B configured
> cable) to + and pins 2 and 4 (orange and blue) -."

| RJ45 pin | T568-B colour | Signal |
|---|---|---|
| 1 | white/orange | RS-485 **A / +** |
| 3 | white/green  | RS-485 **A / +** |
| 2 | orange       | RS-485 **B / −** |
| 4 | blue         | RS-485 **B / −** |
| 5–8 | — | **C and R from the thermostat bus — 24 VAC** |

> ⚠️ **The remaining pins carry 24 VAC.** ccutrer: *"The other pins are C and R
> from the thermostat bus, providing 24VAC power. DO NOT SHORT THESE PINS AGAINST
> ANYTHING..."* Shorting them blows a 3 A automotive fuse at best and can **brick
> the ABC board** at worst.

**Cable rule:** land only two conductors — pin 1 → A and pin 2 → B is enough
(pins 3/4 duplicate the same nets). Leave every other pin **out of the plug
entirely**, so no 24 VAC conductor exists in your cable. Before connecting,
meter for open circuit between your A/B leads and plug pins 5–8; that isolation
check matters more than continuity.

## Serial parameters

**19200 baud, 8 data bits, EVEN parity, 1 stop bit (8E1). Slave address 1**
(address 2 mirrors the same data). FC03 read; FC06 or FC16 write.

Parity is the classic trap — assume 8N1 and you get silence or garbage. The
value is **EVEN**, e.g. ccutrer's README: *"Set up your server (like ser2net) to
use 19200 baud, EVEN."*

## Bus topology — avoiding two masters

If your system has a WaterFurnace **AWL** (Symphony bridge), it is itself a
Modbus master polling the ABC continuously — so naively dropping a second master
onto the same segment causes collisions. The AID port is designed for this; the
AWL provides a pass-through. ccutrer:

> "If you would still like your AWL to function, you can connect AWL to the AID
> port on the heat pump, and then connect your computer to the AID Tool
> pass-through port on the AWL."

```
Aurora ABC  --(AID port, RJ45)-->  AWL  --(AID Tool pass-through port)-->  ESP32
```

**First contact should be passive.** Whether the AWL truly arbitrates or simply
bridges A/B straight through is worth confirming per install. Set the firmware's
`polling_enabled` to `false` (it persists) so the board sits electrically silent,
plug in, and confirm nothing is disturbed before enabling polling. The firmware
also waits for a t3.5 idle gap before every transmit, and exposes
`bus.bus_busy` / `bus.crc_errors` on `/api/health` so contention is visible.

## Transceiver caveat

ccutrer, verbatim: *"Any adapter based on the MAX485 chip is not supported."*
Worth confirming which part your board or adapter uses. The failure modes behind
that warning are 5 V-only parts on a 3.3 V bus, no fail-safe idle biasing, and
manual DE/RE direction control that can't turn around fast enough at 19200 with
tight inter-frame gaps.

## Two gotchas worth knowing (Windows / USB-RS485 bench setup)

Useful if you bring up the firmware against the simulator over a USB-RS485
adapter before touching real hardware:

1. **FTDI adapters may need a driver.** An FT232R (`VID_0403 PID_6001`) can
   enumerate with no COM port (Device Manager "Code 28"). Install the FTDI CDM
   VCP driver (`pnputil /add-driver ...` from an elevated shell), then it appears
   as a COM port.
2. **USB latency batching breaks t3.5 framing.** An FTDI adapter's default 16 ms
   latency timer is ~8× the RTU 3.5-character idle gap (~2 ms at 19200), so a
   single Modbus frame arrives split across reads. Anything that frames by idle
   gap over a USB-serial adapter will misframe unless the latency timer is
   lowered. (The bundled simulator frames by expected request length instead, so
   it is immune.)
