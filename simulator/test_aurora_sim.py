#!/usr/bin/env python3
"""
Offline tests for the Aurora simulator's behavioural model.

No serial port needed - drives the model directly with a fake clock. These
assert behaviours measured on a real WaterFurnace Aurora / IntelliZone-2,
especially the ones that are counter-intuitive enough to fool a client.

    py tools/test_aurora_sim.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from aurora_sim.model import (AuroraModel, EXC_ILLEGAL_ADDRESS, EXC_ILLEGAL_VALUE,
                              decode_zone_config, encode_zone_config)
from aurora_sim.rtu import RtuSlave, with_crc

SNAPSHOT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "sample_snapshot.json")

failures = []


def check(name, got, want):
    ok = got == want
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: got {got!r}" + ("" if ok else f", want {want!r}"))
    if not ok:
        failures.append(name)


def mirror_setpoints(model, zone_n=1):
    """Read the setpoints back through the 31xxx mirror, as a client would."""
    z = model.zones[zone_n - 1]
    vals, exc = model.read(z.read_base, 3)
    assert exc is None
    return decode_zone_config(vals[1], vals[2])


print("== zone config packing (inverse of the zone config-word decode) ==")
for heat, cool in [(70, 76), (68, 72), (40, 99), (90, 54)]:
    hi, lo = encode_zone_config(heat, cool)
    check(f"roundtrip {heat}/{cool}", decode_zone_config(hi, lo), (heat, cool))

print("\n== synthesised IZ2 zone block matches observed values ==")
m = AuroraModel(SNAPSHOT, apply_delay_s=10.0)
check("zone count", len(m.zones), 2)
check("Z1 mirror setpoints", mirror_setpoints(m, 1), (70, 76))
check("Z2 mirror setpoints", mirror_setpoints(m, 2), (68, 72))
vals, _ = m.read(m.zones[0].read_base, 1)
check("Z1 ambient temp x10", vals[0], 740)
check("zone count in reg 31101 bits 8-10", (m.regs[31101] >> 8) & 7, 2)

print("\n== read-only mirror rejects writes with exception 2 ==")
check("write 31008", m.write(31008, 1234, now=0.0), EXC_ILLEGAL_ADDRESS)
check("write 31007", m.write(31007, 700, now=0.0), EXC_ILLEGAL_ADDRESS)

print("\n== setpoint write applies only after the IZ2 sync delay ==")
m = AuroraModel(SNAPSHOT, apply_delay_s=10.0)
check("write 21204=720 accepted", m.write(21204, 720, now=0.0), None)
m.tick(1.0)
check("mirror unchanged 1s after write", mirror_setpoints(m, 1), (70, 76))
m.tick(5.0)
check("mirror unchanged 5s after write", mirror_setpoints(m, 1), (70, 76))
m.tick(11.0)
check("mirror updated 11s after write", mirror_setpoints(m, 1), (70, 72))

print("\n== THE TRAP: out-of-range writes are silently ignored, not clamped ==")
m = AuroraModel(SNAPSHOT, apply_delay_s=1.0)
# A bare 72 means 7.2 F - the classic mis-scaling mistake.
check("write 21204=72 returns NO exception", m.write(21204, 72, now=0.0), None)
m.tick(5.0)
check("  ...and setpoint is unchanged", mirror_setpoints(m, 1), (70, 76))
check("  ...but raw value IS stored in 21xxx", m.read(21204, 1)[0][0], 72)
check("silent_rejects counted", m.stats["silent_rejects"], 1)

check("write 21204=1000 (100F) no exception", m.write(21204, 1000, now=10.0), None)
m.tick(15.0)
check("  ...100F not applied (ceiling is 99)", mirror_setpoints(m, 1), (70, 76))

print("\n== measured cooling boundaries: 53 rejected / 54 applied, 99 ok / 100 not ==")
for raw, want_sp, label in [(530, 76, "53F rejected"), (540, 54, "54F applied"),
                            (990, 99, "99F applied"), (1000, 99, "100F rejected")]:
    m2 = AuroraModel(SNAPSHOT, apply_delay_s=1.0)
    if raw in (990, 1000):
        m2.write(21204, 990, now=0.0)
        m2.tick(2.0)
    m2.write(21204, raw, now=10.0)
    m2.tick(12.0)
    check(label, mirror_setpoints(m2, 1)[1], want_sp)

print("\n== heating range 40-90 F ==")
m = AuroraModel(SNAPSHOT, apply_delay_s=1.0)
m.write(21203, 350, now=0.0); m.tick(2.0)
check("35F heating rejected", mirror_setpoints(m, 1)[0], 70)
m.write(21203, 650, now=10.0); m.tick(12.0)
check("65F heating applied", mirror_setpoints(m, 1)[0], 65)

print("\n== zone 2 uses stride 9 (21213 = Z2 cool) ==")
m = AuroraModel(SNAPSHOT, apply_delay_s=1.0)
m.write(21213, 750, now=0.0); m.tick(2.0)
check("Z2 cool -> 75", mirror_setpoints(m, 2), (68, 75))
check("Z1 untouched", mirror_setpoints(m, 1), (70, 76))

print("\n== schedule reasserts over a written setpoint (temporary hold) ==")
m = AuroraModel(SNAPSHOT, apply_delay_s=1.0, reassert_after_s=30.0)
m.write(21204, 720, now=0.0)
m.tick(2.0)
check("write applied", mirror_setpoints(m, 1), (70, 72))
m.tick(20.0)
check("still held at 20s", mirror_setpoints(m, 1), (70, 72))
m.tick(31.0)
check("schedule reasserted at 31s", mirror_setpoints(m, 1), (70, 76))
check("reassert counted", m.stats["reasserts"], 1)

print("\n== fault register 25: low 15 bits = code, bit 15 = lockout ==")
m = AuroraModel(SNAPSHOT)
m.inject_fault(5, lockout=True)
check("E5 with lockout raw", m.regs[25], 0x8005)
check("decoded", m.current_fault(), (5, True))
check("clear via reg 47 = 21845", m.write(47, 21845, now=0.0), None)
check("  ...fault cleared", m.current_fault(), (0, False))
check("reg 47 with wrong magic -> exception 3", m.write(47, 1, now=0.0), EXC_ILLEGAL_VALUE)

print("\n== unmapped register writes -> exception 2 ==")
check("write 9999", m.write(9999, 1, now=0.0), EXC_ILLEGAL_ADDRESS)

print("\n== RTU layer: exception frame is well formed ==")
m = AuroraModel(SNAPSHOT)
s = RtuSlave(m, addr=1)
resp = s.handle(with_crc(bytes([1, 6, 31008 >> 8, 31008 & 0xFF, 0, 1])), now=0.0)
check("FC06 to mirror -> exception", (resp[1], resp[2]), (0x86, EXC_ILLEGAL_ADDRESS))
resp = s.handle(with_crc(bytes([1, 3, 0, 30, 0, 1])), now=0.0)
check("FC03 reg 30 -> captured value", (resp[3] << 8) | resp[4], 16973)
check("foreign address -> silence", s.handle(with_crc(bytes([2, 3, 0, 30, 0, 1])), now=0.0), None)

print("\n== dynamics: values move, and move for the right reasons ==")
from aurora_sim.dynamics import (Dynamics, SENTINEL_REGS, SENTINEL_RAW,
                                 FP1_TRIP_F, REG_FP1, REG_EWT, REG_PUMP_PCT)


def run_scenario(scenario, ticks=600, step=30.0):
    m = AuroraModel(SNAPSHOT)
    m.attach_dynamics(Dynamics(m, scenario=scenario))
    d = m.dynamics
    start_ewt = d.ewt
    t, starts, prev, fp1_min = 0.0, 0, False, 999.0
    for _ in range(ticks):
        t += step
        m.tick(t)
        if d.running and not prev:
            starts += 1
        if d.compressor > 5.0:
            fp1_min = min(fp1_min, d.fp1)
        prev = d.running
    return m, d, starts, fp1_min, start_ewt


m, d, starts, fp1_min, start_ewt = run_scenario("cooling")
check("cooling cycles the compressor", starts > 0, True)
check("cooling moves entering water temp", abs(d.ewt - start_ewt) > 1.0, True)
check("cooling keeps FP1 well clear of the trip", fp1_min > FP1_TRIP_F + 20, True)
check("cooling does not lock out", d.locked_out, False)

m, d, starts, fp1_min, _ = run_scenario("heating")
check("heating cycles the compressor", starts > 0, True)
check("healthy heating loop stays above the FP1 trip", fp1_min > FP1_TRIP_F, True)
check("healthy heating does not lock out", d.locked_out, False)

m, d, starts, fp1_min, _ = run_scenario("fp1-freeze")
check("starved loop drives FP1 to the trip", fp1_min <= FP1_TRIP_F, True)
check("starved loop locks out", d.locked_out, True)
check("  ...with E5 and the lockout bit", m.current_fault(), (5, True))
check("  ...and stops the compressor", d.compressor, 0.0)
check("  ...and the pump reads 0", m.regs[REG_PUMP_PCT], 0)

print("\n== dynamics under fine-grained ticks (regression) ==")
# The serial link ticks every ~1 ms, not every 30 s. A ramp that only works at
# coarse step sizes is broken where it actually runs, so exercise the small-dt
# path explicitly - this is precisely how the compressor-never-starts bug got in.
m = AuroraModel(SNAPSHOT)
m.attach_dynamics(Dynamics(m, scenario="cooling", time_scale=25.0))
d = m.dynamics
m.zones[1].temp_f = 75.0        # 3 F above Z2's cooling setpoint: real demand
t = 0.0
for _ in range(20000):          # 20 s of wall clock at 1 ms ticks, 25x scaled
    t += 0.001
    m.tick(t)
check("compressor starts with 1 ms ticks", d.compressor > 5.0, True)
check("  ...and the pump follows it", m.regs[REG_PUMP_PCT] > 0, True)
check("  ...within the 50-100% band", 50 <= m.regs[REG_PUMP_PCT] <= 100, True)

print("\n== dynamics fidelity guards ==")
for scen in Dynamics.SCENARIOS:
    m2 = AuroraModel(SNAPSHOT)
    m2.attach_dynamics(Dynamics(m2, scenario=scen))
    reg30_before = m2.regs[30]
    t = 0.0
    for _ in range(120):
        t += 30.0
        m2.tick(t)
    absent = [r for r in SENTINEL_REGS if m2.regs.get(r) != SENTINEL_RAW]
    check(f"{scen}: absent sensors stay absent", absent, [])
    # reg 30 is ASCII ("BM", part of the model string) despite being labelled
    # "Actual Comp Speed" - animating it would emit garbage.
    check(f"{scen}: reg 30 (ASCII model string) untouched", m2.regs[30], reg30_before)

print()
if failures:
    print(f"{len(failures)} FAILURES: {failures}")
    sys.exit(1)
print("all tests passed")
