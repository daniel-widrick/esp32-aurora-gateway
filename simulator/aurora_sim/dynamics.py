"""
Physics-lite dynamics so the simulated furnace actually runs.

Design rules, in priority order:

1. **Never fabricate an absent sensor.** Registers 1112, 1113 and 1116 read the
   ABC's -999.9 sentinel in the live capture, meaning the sensor isn't present
   on this unit. They stay at the sentinel forever. A simulator that starts
   producing a plausible leaving-air temperature would train the client to
   expect data the real machine never sends.
2. **Only drive registers whose scaling we're confident about.** Anything with
   an ambiguous label (the compressor status words at 1107/1108, for instance)
   is left frozen at its captured value rather than animated on a guess. Note
   reg 30 is labelled "Actual Comp Speed" but holds ASCII ("BM") - it's part of
   the model string, so it is emphatically not driven here.
3. **Start from the capture.** Every driven register begins at its real value
   and moves from there, so a client that trusts the snapshot stays happy.

The refrigerant-circuit relationships follow the unit's own observed behaviour:
the variable-speed loop pump tracks compressor speed (50-100%), and FP1 is the
*source*-side freeze sensor - which is why it is the one that trips in winter
heating, not in cooling.
"""

from __future__ import annotations

import math

# Sensors absent on this unit. Never written. (0xD8F1 = -9999 = -999.9 F)
SENTINEL_REGS = (1112, 1113, 1116)
SENTINEL_RAW = 0xD8F1

# Registers this module drives, with their scaling.
REG_FP1 = 19            # source-side freeze sensor, tenths F
REG_FP2 = 20            # load-side freeze sensor, tenths F
REG_FAULT = 25
REG_FAN_SPEED = 54
REG_PUMP_PCT = 325      # VS pump output, whole %
REG_LWT = 1110          # leaving water, tenths F
REG_EWT = 1111          # entering water, tenths F
REG_HOT_WATER = 1114    # desuperheater, tenths F
REG_DISCH_PRESS = 1115  # tenths psi
REG_WATERFLOW = 1117    # tenths gpm
REG_DISCH_PRESS2 = 3322
REG_SUCTION_PRESS = 3323
REG_DISCH_TEMP = 3325
REG_COMP_AMBIENT = 3326
REG_VS_DRIVE_EWT = 3330
REG_COMP_POWER = 3422

# FP1 lockout threshold. The DIP-selected value reads 0xFF in reg 33 (likely a
# default we never confirmed physically), so this uses the standard 30 F water
# setting rather than the 15 F antifreeze setting.
FP1_TRIP_F = 30.0
ECODE_FP1 = 5           # E5 - the chronic fault on this unit


def _lag(current: float, target: float, dt: float, tau: float) -> float:
    """First-order approach to `target` with time constant `tau` seconds."""
    if tau <= 0:
        return target
    return current + (target - current) * (1.0 - math.exp(-dt / tau))


class Dynamics:
    """
    Scenarios:
      steady      - captured values with sensor noise; no cycling
      cooling     - summer cooling, compressor cycles on zone demand
      heating     - winter heating
      fp1-freeze  - heating into a starved/too-cold loop until E5 locks out
    """

    SCENARIOS = ("steady", "cooling", "heating", "fp1-freeze")

    def __init__(self, model, scenario="steady", time_scale=1.0, rng=None):
        if scenario not in self.SCENARIOS:
            raise ValueError(f"unknown scenario {scenario!r}; pick from {self.SCENARIOS}")
        self.model = model
        self.scenario = scenario
        self.time_scale = time_scale
        self.rng = rng or model._rng

        # Continuous state, seeded from the capture so we start where it did.
        self.compressor = 0.0          # 0, else 40-100 %
        self.running = False
        self.ewt = self._tenths(REG_EWT, 68.3)
        self.lwt = self._tenths(REG_LWT, 73.0)
        self.fp1 = self._tenths(REG_FP1, 74.4)
        self.fp2 = self._tenths(REG_FP2, 52.8)
        self.hot_water = self._tenths(REG_HOT_WATER, 93.8)
        self.disch_temp = self._tenths(REG_DISCH_TEMP, 94.7)
        self.comp_ambient = self._tenths(REG_COMP_AMBIENT, 80.1)

        self.outdoor_f = 88.0 if scenario in ("steady", "cooling") else 22.0
        # Undisturbed loop temperature at rest. A healthy closed loop sits in
        # the 40s in winter; the freeze scenario starts colder.
        if scenario in ("steady", "cooling"):
            self.ground_f = 68.0
        elif scenario == "fp1-freeze":
            self.ground_f = 40.0
        else:
            self.ground_f = 46.0
        # How far under soil temp the loop is dragged at full load. A healthy
        # loop stays close; an undersized / too-cold / marginally-antifreezed
        # one collapses - the leading suspect for the chronic
        # E5. That's what fp1-freeze models: the soil is fine, the loop simply
        # can't sustain the extraction rate.
        self.loop_penalty = 34.0 if scenario == "fp1-freeze" else 8.0

        self.locked_out = False
        self._elapsed = 0.0

    def _tenths(self, reg, default):
        raw = self.model.regs.get(reg)
        if raw is None:
            return default
        return ((raw - 65536) if raw > 32767 else raw) / 10.0

    def _put_tenths(self, reg, value_f):
        assert reg not in SENTINEL_REGS, f"refusing to write absent-sensor reg {reg}"
        self.model.regs[reg] = int(round(value_f * 10)) & 0xFFFF

    def _noise(self, scale=0.05):
        return self.rng.uniform(-scale, scale)

    # -- demand ------------------------------------------------------------

    def _update_demand(self, dt):
        """Cycle the compressor against zone demand with a deadband."""
        if self.scenario == "steady":
            return
        if self.locked_out:
            self.running = False
            self.compressor = _lag(self.compressor, 0.0, dt, 8.0)
            if self.compressor < 3.0:
                self.compressor = 0.0     # else the pump never reads 0
            return

        cooling = self.scenario == "cooling"
        want = False
        error = 0.0
        for z in self.model.zones:
            if cooling:
                err = z.temp_f - z.cool_sp
            else:
                err = z.heat_sp - z.temp_f
            error = max(error, err)
            # 0.5 F deadband, with hysteresis via the current running state.
            if err > (0.2 if self.running else 0.6):
                want = True

        self.running = want
        if want:
            # Variable-speed: floor of 40%, rising with how far off we are.
            target = min(100.0, 40.0 + error * 22.0)
        else:
            target = 0.0
        self.compressor = _lag(self.compressor, target, dt, 12.0)
        # Snap to a clean zero only when we're actually commanded off. Doing it
        # unconditionally is a trap: with fine-grained ticks (the serial link
        # ticks every ~1 ms, not every 30 s) each ramp-up step is a fraction of
        # a percent, so a blanket "< 3 -> 0" resets the ramp every tick and the
        # compressor can never start at all.
        if target == 0.0 and self.compressor < 3.0:
            self.compressor = 0.0

    # -- loop and refrigerant circuit --------------------------------------

    def _update_loop(self, dt):
        load = self.compressor / 100.0
        cooling = self.scenario == "cooling" or self.scenario == "steady"

        # Entering water pulls toward soil temp, dragged away by the heat we're
        # rejecting into (cooling) or extracting from (heating) the loop. With
        # no flow the loop equalises with soil slowly instead of tracking load,
        # so a stopped unit cannot freeze its own source.
        if cooling:
            ewt_target = self.ground_f + 14.0 * load
        else:
            ewt_target = self.ground_f - self.loop_penalty * load
        self.ewt = _lag(self.ewt, ewt_target, dt, 180.0 if load > 0.05 else 1800.0)

        # Leaving water differs from entering by the heat exchanged.
        delta = 9.0 * load
        self.lwt = _lag(self.lwt, self.ewt + (delta if cooling else -delta), dt, 60.0)

        # FP1 watches the SOURCE side, FP2 the LOAD side. In cooling the source
        # is the warm condenser loop and the load coil runs cold; in heating
        # they swap, which is exactly why FP1 is the one that freezes in winter.
        if cooling:
            self.fp1 = _lag(self.fp1, self.lwt + 1.5, dt, 45.0)
            self.fp2 = _lag(self.fp2, 52.0 - 8.0 * load, dt, 45.0)
        else:
            self.fp1 = _lag(self.fp1, self.lwt - 6.0 * load, dt, 45.0)
            self.fp2 = _lag(self.fp2, 95.0 + 15.0 * load, dt, 45.0)

        self.disch_temp = _lag(self.disch_temp, 95.0 + 85.0 * load, dt, 40.0)
        self.comp_ambient = _lag(self.comp_ambient, self.outdoor_f - 6.0 + 12.0 * load,
                                 dt, 300.0)
        # Desuperheater only makes hot water while the compressor runs, and is
        # capped by the 130 F high limit that produces the benign E15.
        hw_target = 130.0 if load > 0.1 else 92.0
        self.hot_water = _lag(self.hot_water, hw_target, dt, 400.0)

    def _update_zones(self, dt):
        """Move room temperatures under the equipment's influence."""
        if self.scenario == "steady":
            return
        cooling = self.scenario == "cooling"
        load = self.compressor / 100.0
        for z in self.model.zones:
            # Envelope drags the room toward outdoor; the unit pushes back.
            # These constants matter: the equipment must out-pull the envelope
            # by a clear margin or the unit never satisfies and never cycles,
            # which is both unrealistic and useless for testing a client.
            # ~4 F/hr envelope loss at a 48 F delta, ~11 F/hr of capacity.
            drift = (self.outdoor_f - z.temp_f) / 40000.0
            effect = (-load * 0.0030) if cooling else (load * 0.0030)
            z.temp_f += (drift + effect) * dt
            z.temp_f = max(45.0, min(95.0, z.temp_f))

    def _update_protection(self, dt):
        """Trip E5 when FP1 falls below the freeze threshold."""
        if self.locked_out:
            return
        # Protection is evaluated in every scenario, so whether a loop freezes
        # is a consequence of the physics rather than a flag. A healthy heating
        # loop simply never gets FP1 down to the threshold; the starved one
        # does. That distinction is the whole point of the scenario.
        #
        # E5 is also an operating fault: a unit sitting idle cannot trip its
        # own source-side freeze protection, so require the compressor running.
        if self.compressor <= 5.0:
            return
        if self.fp1 <= FP1_TRIP_F:
            self.locked_out = True
            self.model.inject_fault(ECODE_FP1, lockout=True)

    # -- publish -------------------------------------------------------------

    def _publish(self):
        load = self.compressor / 100.0

        self._put_tenths(REG_EWT, self.ewt + self._noise())
        self._put_tenths(REG_VS_DRIVE_EWT, self.ewt + self._noise())
        self._put_tenths(REG_LWT, self.lwt + self._noise())
        self._put_tenths(REG_FP1, self.fp1 + self._noise())
        self._put_tenths(REG_FP2, self.fp2 + self._noise())
        self._put_tenths(REG_HOT_WATER, self.hot_water + self._noise())
        self._put_tenths(REG_DISCH_TEMP, self.disch_temp + self._noise(0.2))

        # The VS loop pump tracks compressor speed over a 50-100% band
        # (observed) and idles off when the compressor is off.
        pump = 0 if self.compressor <= 0 else int(round(50 + 50 * load))
        self.model.regs[REG_PUMP_PCT] = pump & 0xFFFF
        self._put_tenths(REG_WATERFLOW, 0.0 if pump == 0 else 4.0 + 5.0 * (pump / 100.0))

        # Pressures track load; both discharge registers move together, as they
        # did in the capture (1115=2182, 3322=2180).
        disch = 120.0 + 190.0 * load + self._noise(1.0)
        self._put_tenths(REG_DISCH_PRESS, disch)
        self._put_tenths(REG_DISCH_PRESS2, disch - 0.2)
        self._put_tenths(REG_SUCTION_PRESS, 150.0 - 22.0 * load + self._noise(0.5))

        self.model.regs[REG_COMP_POWER] = int(round(3800 * load)) & 0xFFFF
        self.model.regs[REG_FAN_SPEED] = 0 if load == 0 else max(1, int(round(11 * load)))

    # -- entry point ---------------------------------------------------------

    def update(self, dt: float):
        dt *= self.time_scale
        if dt <= 0:
            return
        self._elapsed += dt

        self._update_demand(dt)
        self._update_loop(dt)
        self._update_zones(dt)
        self._update_protection(dt)
        self._publish()

    def summary(self) -> str:
        code, lock = self.model.current_fault()
        return (f"{self.scenario:11s} t={self._elapsed:6.0f}s comp={self.compressor:5.1f}% "
                f"EWT={self.ewt:5.1f} LWT={self.lwt:5.1f} FP1={self.fp1:5.1f} "
                f"FP2={self.fp2:5.1f} pump={self.model.regs.get(REG_PUMP_PCT, 0):3d}% "
                f"Z1={self.model.zones[0].temp_f:4.1f} "
                f"fault={'E%d%s' % (code, ' LOCKOUT' if lock else '') if code else '-'}")
