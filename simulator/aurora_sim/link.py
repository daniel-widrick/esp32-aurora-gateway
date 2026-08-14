"""
Serial RS-485 transport for the Aurora simulator, with deliberate impairments
and frame statistics.

The impairments exist so the bench can answer questions the happy path can't:
does the master retry, does it resync after a corrupt frame, and - the reason
this was built - does a physical cable carry tens of thousands of frames with
*zero* errors, or does it fail one time in ten thousand the way a marginal
crimp does?
"""

from __future__ import annotations

import time

import serial

from .rtu import RtuSlave, expected_request_len, check_crc

BAUD = 19200
BITS_PER_CHAR = 11              # 8E1: start + 8 data + parity + stop
CHAR_TIME = BITS_PER_CHAR / BAUD
T3_5 = CHAR_TIME * 3.5          # ~2.0 ms
STALE_AFTER = 0.25              # drop an unparseable partial after this silence


class Stats:
    def __init__(self):
        self.requests = 0
        self.responses = 0
        self.bad_crc = 0
        self.stale_partials = 0
        self.dropped = 0
        self.corrupted = 0
        self.silent_addr = 0
        self.started = time.monotonic()

    def as_dict(self):
        elapsed = max(time.monotonic() - self.started, 1e-9)
        d = {k: v for k, v in vars(self).items() if not k.startswith("_") and k != "started"}
        d["elapsed_s"] = round(elapsed, 1)
        d["req_per_s"] = round(self.requests / elapsed, 2)
        total = self.requests + self.bad_crc + self.stale_partials
        d["error_rate"] = round((self.bad_crc + self.stale_partials) / total, 6) if total else 0.0
        return d

    def line(self):
        d = self.as_dict()
        return (f"[stats] req={d['requests']} resp={d['responses']} "
                f"bad_crc={d['bad_crc']} stale={d['stale_partials']} "
                f"dropped={d['dropped']} corrupted={d['corrupted']} "
                f"rate={d['req_per_s']}/s err={d['error_rate']}")


class SerialLink:
    def __init__(self, port, slave: RtuSlave, verbose=False, impair=None,
                 stats_every=0.0):
        self.slave = slave
        self.verbose = verbose
        self.impair = impair or {}
        self.stats = Stats()
        self.stats_every = stats_every
        self._last_stats = time.monotonic()

        self.ser = serial.Serial(
            port=port,
            baudrate=BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,      # 8E1 - not 8N1
            stopbits=serial.STOPBITS_ONE,
            timeout=CHAR_TIME * 1.5,
        )
        self._rng = self.slave.model._rng

    def _maybe_impair(self, response: bytes):
        """Return the bytes to actually send, or None to stay silent."""
        if self._rng.random() < self.impair.get("drop_rate", 0.0):
            self.stats.dropped += 1
            if self.verbose:
                print("  ~~ dropping response (injected)", flush=True)
            return None
        if self._rng.random() < self.impair.get("corrupt_rate", 0.0):
            data = bytearray(response)
            idx = self._rng.randrange(len(data))
            data[idx] ^= 1 << self._rng.randrange(8)
            self.stats.corrupted += 1
            if self.verbose:
                print(f"  ~~ corrupting byte {idx} of response (injected)", flush=True)
            return bytes(data)
        return response

    def run(self):
        buf = bytearray()
        last_rx = time.monotonic()

        while True:
            chunk = self.ser.read(256)
            now = time.monotonic()
            if chunk:
                buf.extend(chunk)
                last_rx = now

            while buf:
                need = expected_request_len(bytes(buf))
                if need is None:
                    break
                if need == -1:
                    del buf[0]
                    continue
                if len(buf) < need:
                    break
                frame = bytes(buf[:need])
                if not check_crc(frame):
                    self.stats.bad_crc += 1
                    if self.verbose:
                        print(f"  !! bad CRC, resyncing on {frame.hex(' ')}", flush=True)
                    del buf[0]
                    continue
                del buf[:need]
                self._serve(frame, now)

            if buf and (now - last_rx) > STALE_AFTER:
                self.stats.stale_partials += 1
                if self.verbose:
                    print(f"  -- discarding stale partial {bytes(buf).hex(' ')}", flush=True)
                buf.clear()

            self.slave.model.tick(now)

            if self.stats_every and (now - self._last_stats) >= self.stats_every:
                print(self.stats.line(), flush=True)
                self._last_stats = now

    def _serve(self, frame: bytes, now: float):
        self.stats.requests += 1
        if self.verbose:
            print(f"<- {frame.hex(' ')}", flush=True)

        response = self.slave.handle(frame, now)
        if response is None:
            self.stats.silent_addr += 1
            return

        if self.verbose:
            req = self.slave.last_request
            if req:
                print(f"   {req[0]} {req[1]}"
                      + (f" x{req[2]}" if req[0] == "read" else f" = {req[2]}"), flush=True)

        latency = self.impair.get("latency_ms", 0.0) / 1000.0
        time.sleep(max(T3_5, latency))

        out = self._maybe_impair(response)
        if out is None:
            return
        self.ser.write(out)
        self.ser.flush()
        self.stats.responses += 1
        if self.verbose:
            print(f"-> {out.hex(' ')}\n", flush=True)

    def close(self):
        self.ser.close()
