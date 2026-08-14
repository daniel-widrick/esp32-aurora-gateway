# Aurora simulator

Impersonates a WaterFurnace 7 Series (NVV048) Aurora ABC / IntelliZone-2 over
RS-485 Modbus-RTU, so the gateway firmware and any client can be developed and
tested with nothing connected to the heat pump.

```powershell
py -m aurora_sim --port COM4 -v                          # replay the capture
py -m aurora_sim --port COM4 --scenario cooling          # a running unit
py -m aurora_sim --port COM4 --scenario fp1-freeze --time-scale 60
py -m aurora_sim --port COM4 --stats-every 5 --corrupt-rate 0.01
py tools/test_aurora_sim.py                              # 60+ offline assertions
```

## Layers

| Module | Role |
|---|---|
| `model.py` | Registers, IZ2 write semantics, faults. **No transport.** |
| `dynamics.py` | Physics-lite: the unit actually runs, cycles and faults |
| `rtu.py` | Modbus-RTU codec and slave dispatch. **No I/O.** |
| `link.py` | RS-485 serial transport, impairments, frame statistics |

The split is deliberate: the behavioural model has no idea it's on a serial
port, so the same core can sit behind a different front end when this becomes a
honeypot module. Only `link.py` would be replaced.

## Fidelity rules

These are what separate a useful simulator from a misleading one:

1. **Absent sensors stay absent.** Registers 1112, 1113 and 1116 read the ABC's
   `-999.9` sentinel (`0xD8F1`) on this unit — the sensors aren't fitted. They
   are never written, and a test asserts it in every scenario. A simulator that
   started emitting a plausible leaving-air temperature would teach a client to
   expect data the real machine never sends.
2. **Only animate registers whose scaling we trust.** Reg 30 is labelled
   "Actual Comp Speed" but contains ASCII (`0x424D` = `"BM"`, part of the model
   string, like reg 105). It is explicitly left alone — a guarded test asserts
   it never changes.
3. **Everything starts from a register capture** (`sample_snapshot.json`) and
   moves from there. The bundled sample is synthetic — plausible values for a
   unit at rest, not a capture from any specific machine. Point the loader at
   your own capture to replay real data.
4. **Unmeasured behaviour is omitted, not invented.** The IZ2 schedule's
   internals were never recovered, so only its *observed* effect is modelled
   (it reasserts a written setpoint), and the reassert cadence is off by
   default because that cadence was never measured.

The IZ2 zone block is synthesised — a capture that lacks the `21xxx`/`31xxx`
registers is seeded from default zone values (Z1 74 °F set 70/76, Z2 72 °F set
68/72) and packed with the exact inverse of the zone config-word decode.

## Write semantics

Modelled from behaviour observed on a real unit, including the trap:

- `21203 + 9·(z−1)` heating, `21204 + 9·(z−1)` cooling, value **°F × 10**.
- **Out-of-range writes are silently ignored** — no Modbus exception, the raw
  value *is* stored in the `21xxx` register, and the setpoint keeps its prior
  value rather than clamping. Boundaries: cooling 54–99 °F, heating 40–90 °F.
- Writes apply on the **IZ2 sync cycle** (`--apply-delay`, default 12 s), not
  instantly.
- The `31xxx` mirror is **read-only** — writes return exception 2.
- `47 = 21845` clears fault history.
- `--reassert-after` makes the schedule take a written setpoint back, so you can
  test that poll-and-re-assert control actually survives.

## Scenarios

| `--scenario` | Behaviour |
|---|---|
| `steady` | Captured values with sensor noise; no cycling (default) |
| `cooling` | Summer cooling; compressor cycles on zone demand |
| `heating` | Winter heating on a **healthy** loop |
| `fp1-freeze` | Heating into a starved loop until E5 locks out |

`--time-scale X` runs the physics X times faster than real time.

Whether a loop freezes is a **consequence of the physics, not a flag**:
protection is evaluated in every scenario. A healthy heating loop bottoms out
around 34 °F FP1 and never trips; the starved one reaches the 30 °F threshold
and locks out with E5 and the lockout bit — reproducing the chronic fault in
a real unit, in August, on a bench. FP1 is the *source*-side sensor, which is
why it trips in heating and stays warm in cooling.

E5 is treated as an operating fault: an idle unit can't trip its own freeze
protection, so the compressor must be running.

## Impairments

`--latency-ms`, `--drop-rate`, `--corrupt-rate` inject response latency, dropped
responses and single-bit corruption; `--stats-every` prints frame counters and
an error rate. This is the instrument for **cable validation**: terminate the
finished RJ45 cable into a breakout in the middle of a known-good link and look
for zero errors over tens of thousands of frames. A marginal crimp passes a
continuity beep and then fails one frame in ten thousand.

## A gotcha worth remembering

`link.py` frames by **expected request length**, not the RTU 3.5-character idle
gap. A USB-serial adapter batches bytes on its latency timer (16 ms by default
on FTDI parts) — about 8× t3.5 at 19200 — so gap-based framing shreds single
requests into garbage. Anything else that sniffs this bus through a USB adapter
will hit the same wall unless the latency timer is lowered.
