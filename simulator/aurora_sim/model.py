"""
Behavioural model of the WaterFurnace Aurora ABC / IntelliZone-2.

Transport-free on purpose: this class knows registers and furnace behaviour and
nothing about serial ports or Modbus framing, so the same model can sit behind
the bench RTU link, a TCP front end, or a honeypot.

Everything modelled here is either replayed from a register capture
(sample_snapshot.json) or reproduces behaviour observed on a real WaterFurnace
Aurora / IntelliZone-2 unit. Where a behaviour was NOT observed it is left out
rather than invented - a simulator that lies is worse than none.

The register map, scaling, and write semantics are cross-checked against the
open-source ccutrer/waterfurnace_aurora project
(https://github.com/ccutrer/waterfurnace_aurora).
"""

from __future__ import annotations

import json
import os
import random

# ---------------------------------------------------------------------------
# Register layout (zone read base 31007 stride 3; write base 21202 stride 9)
# ---------------------------------------------------------------------------

ZONE_READ_BASE = 31007      # + 3*(z-1): ambient temp; +1/+2 packed config words
ZONE_CFG_REG = 31101        # bits 8-10 = zone count
ZONE_WRITE_BASE = 21202     # + 9*(z-1): heat mode, heat SP, cool SP, fan mode

REG_HEAT_MODE = 21202
REG_HEAT_SP = 21203
REG_COOL_SP = 21204
REG_FAN_MODE = 21205
ZONE_WRITE_STRIDE = 9

REG_FAULT = 25              # low 15 bits = stored E-code, bit15 = lockout
REG_CLEAR_FAULTS = 47
CLEAR_FAULTS_MAGIC = 21845  # 0x5555

# Measured boundaries. Cooling probed directly (53 rejected / 54 applied,
# 99 applied / 100 rejected); heating range per ccutrer.
COOL_RANGE_F = (54, 99)
HEAT_RANGE_F = (40, 90)

EXC_ILLEGAL_FUNCTION = 0x01
EXC_ILLEGAL_ADDRESS = 0x02
EXC_ILLEGAL_VALUE = 0x03

# The IZ2 applies a setpoint on its sync cycle, not instantly. Measured at
# roughly 10-20 s; a readback sooner than that still shows the old value, which
# repeatedly looked like "the write did nothing".
DEFAULT_APPLY_DELAY_S = 12.0


def encode_zone_config(heat_sp_f: int, cool_sp_f: int) -> tuple[int, int]:
    """
    Pack a zone's setpoints into the (hi, lo) config word pair.

    Inverse of the zone config-word decode:
        data   = (hi << 16) | lo
        heatSP = ((data >> 11) & 0x3F) + 36
        coolSP = ((data >> 17) & 0x3F) + 36
    """
    data = (((heat_sp_f - 36) & 0x3F) << 11) | (((cool_sp_f - 36) & 0x3F) << 17)
    return (data >> 16) & 0xFFFF, data & 0xFFFF


def decode_zone_config(hi: int, lo: int) -> tuple[int, int]:
    """(heat_sp_f, cool_sp_f) from a zone's packed config-word pair."""
    data = (hi << 16) | lo
    return ((data >> 11) & 0x3F) + 36, ((data >> 17) & 0x3F) + 36


class Zone:
    """One IntelliZone-2 zone."""

    def __init__(self, number: int, temp_f: float, heat_sp: int, cool_sp: int):
        self.number = number
        self.temp_f = temp_f
        self.heat_sp = heat_sp
        self.cool_sp = cool_sp
        # What the IZ2's own schedule wants. A Modbus setpoint write is a
        # temporary hold; the schedule reasserts its value later (observed live:
        # a written 72 had drifted back to 74). reassert_after_s = None disables
        # that, since the real cadence was never measured - only the fact of it.
        self.sched_heat_sp = heat_sp
        self.sched_cool_sp = cool_sp

    @property
    def read_base(self) -> int:
        return ZONE_READ_BASE + 3 * (self.number - 1)

    @property
    def write_base(self) -> int:
        return ZONE_WRITE_BASE + ZONE_WRITE_STRIDE * (self.number - 1)


class AuroraModel:
    def __init__(self, snapshot_path: str, zones=None, apply_delay_s=DEFAULT_APPLY_DELAY_S,
                 reassert_after_s=None, seed=1):
        with open(snapshot_path) as fh:
            snap = json.load(fh)
        self.captured_at = snap.get("timestamp", "?")
        self.slave_addr = int(snap.get("addr", 1))
        self.regs: dict[int, int] = {int(k): int(v) & 0xFFFF for k, v in snap["regs"].items()}

        self.apply_delay_s = apply_delay_s
        self.reassert_after_s = reassert_after_s
        self._rng = random.Random(seed)

        # Pending setpoint applies: list of (due_time, zone, field, value_f).
        self._pending: list[tuple[float, Zone, str, int]] = []
        # When each zone's setpoint was last overridden by a write.
        self._held_since: dict[tuple[int, str], float] = {}

        # The capture contains no 21xxx or 31xxx registers at all - the AWL web
        # UI never read that block in the snapshot we took - so the whole IZ2
        # zone area is synthesised here from separately observed values
        # (Z1 74F set 70/76, Z2 72F set 68/72).
        if zones is None:
            zones = [(74.0, 70, 76), (72.0, 68, 72)]
        self.zones = [Zone(i + 1, t, h, c) for i, (t, h, c) in enumerate(zones)]
        self._write_zone_block()

        self.stats = {"reads": 0, "writes": 0, "rejected_writes": 0,
                      "silent_rejects": 0, "applies": 0, "reasserts": 0}

        # Optional Dynamics instance (see dynamics.py). Attached rather than
        # constructed here so the model stays usable on its own - the offline
        # tests drive setpoint semantics with no dynamics at all.
        self.dynamics = None
        self._last_tick = None

    def attach_dynamics(self, dynamics):
        self.dynamics = dynamics

    # -- register block maintenance -----------------------------------------

    def _write_zone_block(self):
        cfg = self.regs.get(ZONE_CFG_REG, 0)
        cfg = (cfg & ~(7 << 8)) | ((len(self.zones) & 7) << 8)
        self.regs[ZONE_CFG_REG] = cfg

        for z in self.zones:
            hi, lo = encode_zone_config(z.heat_sp, z.cool_sp)
            self.regs[z.read_base] = int(round(z.temp_f * 10)) & 0xFFFF
            self.regs[z.read_base + 1] = hi
            self.regs[z.read_base + 2] = lo
            # The 21xxx write block mirrors the applied setpoints in F x 10.
            self.regs.setdefault(z.write_base + 1, z.heat_sp * 10)
            self.regs.setdefault(z.write_base + 2, z.cool_sp * 10)

    def zone_for_write_reg(self, reg: int):
        """Return (zone, field) if `reg` is a per-zone write register."""
        if reg < ZONE_WRITE_BASE:
            return None, None
        offset = reg - ZONE_WRITE_BASE
        idx, field_off = divmod(offset, ZONE_WRITE_STRIDE)
        if idx >= len(self.zones):
            return None, None
        field = {0: "heat_mode", 1: "heat_sp", 2: "cool_sp", 3: "fan_mode"}.get(field_off)
        if field is None:
            return None, None
        return self.zones[idx], field

    # -- reads ---------------------------------------------------------------

    def read(self, start: int, count: int, strict: bool = False):
        """Return (values, exception). Unmapped -> 0 unless strict."""
        values = []
        for reg in range(start, start + count):
            if reg in self.regs:
                values.append(self.regs[reg])
            elif strict:
                return None, EXC_ILLEGAL_ADDRESS
            else:
                values.append(0)
        self.stats["reads"] += 1
        return values, None

    # -- writes --------------------------------------------------------------

    def write(self, reg: int, value: int, now: float):
        """
        Apply Modbus write semantics. Returns an exception code, or None.

        The important and counter-intuitive part: an out-of-range setpoint is
        NOT rejected on the wire. The 21xxx register stores the raw value and
        the IZ2 simply declines to apply it - no exception, prior value kept,
        not clamped. A write that "succeeds" and does nothing is the single
        most likely way to fool a client, so it is modelled exactly.
        """
        # Read-only mirror.
        if 31000 <= reg <= 31999:
            self.stats["rejected_writes"] += 1
            return EXC_ILLEGAL_ADDRESS

        if reg == REG_CLEAR_FAULTS:
            if value == CLEAR_FAULTS_MAGIC:
                self.clear_faults()
                self.regs[reg] = value
                self.stats["writes"] += 1
                return None
            self.stats["rejected_writes"] += 1
            return EXC_ILLEGAL_VALUE

        zone, field = self.zone_for_write_reg(reg)
        if zone is None:
            self.stats["rejected_writes"] += 1
            return EXC_ILLEGAL_ADDRESS

        # The raw value is stored regardless - verified on the real unit.
        self.regs[reg] = value & 0xFFFF
        self.stats["writes"] += 1

        if field in ("heat_mode", "fan_mode"):
            return None

        sp_f = value / 10.0
        lo, hi = HEAT_RANGE_F if field == "heat_sp" else COOL_RANGE_F
        if not (lo <= sp_f <= hi):
            # Silently ignored. This is the trap: no exception, no clamp.
            self.stats["silent_rejects"] += 1
            return None

        self._pending.append((now + self.apply_delay_s, zone, field, int(round(sp_f))))
        self._held_since[(zone.number, field)] = now
        return None

    # -- time-dependent behaviour -------------------------------------------

    def tick(self, now: float):
        """Advance deferred applies, schedule reassertion, and dynamics."""
        dt = 0.0 if self._last_tick is None else max(0.0, now - self._last_tick)
        self._last_tick = now

        still_pending = []
        for due, zone, field, sp in self._pending:
            if now >= due:
                setattr(zone, field, sp)
                self.stats["applies"] += 1
            else:
                still_pending.append((due, zone, field, sp))
        self._pending = still_pending

        if self.reassert_after_s is not None:
            for (znum, field), held in list(self._held_since.items()):
                if now - held < self.reassert_after_s:
                    continue
                zone = self.zones[znum - 1]
                sched = getattr(zone, "sched_" + field)
                if getattr(zone, field) != sched:
                    setattr(zone, field, sched)
                    self.stats["reasserts"] += 1
                del self._held_since[(znum, field)]

        if self.dynamics is not None:
            self.dynamics.update(dt)

        self._write_zone_block()

    # -- faults --------------------------------------------------------------

    def inject_fault(self, ecode: int, lockout: bool = False):
        """reg 25: low 15 bits = stored E-code, bit 15 = lockout."""
        self.regs[REG_FAULT] = (ecode & 0x7FFF) | (0x8000 if lockout else 0)

    def clear_faults(self):
        self.regs[REG_FAULT] = 0

    def current_fault(self) -> tuple[int, bool]:
        v = self.regs.get(REG_FAULT, 0)
        return v & 0x7FFF, bool(v & 0x8000)
